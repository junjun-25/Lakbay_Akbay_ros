#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import Joy
from rclpy.duration import Duration

class ArmControllerSwitchNode(Node):
    def __init__(self):
        super().__init__('arm_controller_switch')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.last_joy_time = self.get_clock().now()
        self.joy_timeout = 0.5  # seconds

        self.controller_active = False


        # Subscriber: joy
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )

        # Service client: switch_controller_client

        self.switch_controller_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller',
        )

        # Timer: timer_1
        self.timer_1_timer = self.create_timer(
            0.2,
            self.timer_1_callback,
        )

    
    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()

    def timer_1_callback(self):
        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9

        if elapsed > self.joy_timeout:
            if self.controller_active:
                self.switch_controller(True)
        else:
            if not self.controller_active:
                self.switch_controller(False)

    def switch_controller(self, start: bool):
        # Wait longer for service to be available
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("switch_controller service not available")
            return

        req = SwitchController.Request()

        if start:
            req.activate_controllers = ['arm_controller']
            req.deactivate_controllers = []
        else:
            req.activate_controllers = []
            req.deactivate_controllers = ['arm_controller']

        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = False
        req.timeout = Duration(seconds=0.0).to_msg()

        future = self.switch_controller_client.call_async(req)
        future.add_done_callback(self.switch_done_callback)

    def switch_done_callback(self, future):
        try:
            response = future.result()
            if response.ok:
                self.controller_active = not self.controller_active
                state = "started" if not self.controller_active else "stopped"
                self.get_logger().info(f"Controller {state}")
            else:
                self.get_logger().error(f"Switch controller failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmControllerSwitchNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()