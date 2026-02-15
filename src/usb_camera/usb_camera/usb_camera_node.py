#!/usr/bin/env python3
import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
import cv2

ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)


        if not self.cap.isOpened():
            self.get_logger().error("Could not open USB camera")
            return

        # Publisher: camera_node
        self.camera_node_pub = self.create_publisher(
            Image,
            'camera_node',
            QoSProfile(
                depth=50,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
            )
        )

        # Timer: timer_1
        self.timer_1_timer = self.create_timer(
            1.0/30.0,
            self.timer_1_callback,
        )

    
    def timer_1_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Failed to capture image")
            return

        # Convert OpenCV image to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "usb_camera_frame"

        self.camera_node_pub.publish(msg)

        self.get_logger().info("Published image")
    


def main(args=None):
    rclpy.init(args=args)
    try:
        node = UsbCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()
