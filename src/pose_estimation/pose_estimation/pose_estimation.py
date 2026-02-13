import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import cv2
from cv_bridge import CvBridge


class RobotPoseEstimationNode(Node):
    def __init__(self):
        self.get_logger().info(f"Initializing {self.get_name()}...")


        self.timer_1_timer = self.create_timer(
            0.2,
            self.timer_1_callback,
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RobotPoseEstimationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()
