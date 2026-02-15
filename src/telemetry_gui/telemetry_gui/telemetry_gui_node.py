#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy
from sensor_msgs.msg import JointState
import numpy as np
import sys
from .telemetry_ui import Ui_MainWindow
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from PyQt5 import QtGui
from PyQt5 import QtWidgets
from PyQt5 import QtCore
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class TelemetryGuiNode(Node, QtCore.QObject):
    joint_update_signal = QtCore.pyqtSignal(dict, dict)
    camera_update_signal = QtCore.pyqtSignal(np.ndarray)  # Signal for image updates
    def __init__(self, ui):
        Node.__init__(self, 'telemetry_gui')
        QtCore.QObject.__init__(self)
        self.get_logger().info(f"Initializing {self.get_name()}...")
        self.ui = ui
        self.bridge = CvBridge()  
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_update_signal.connect(self.update_gui)
        self.camera_update_signal.connect(self.update_camera_view)

        # Subscriber: encoder_readings
        self.encoder_readings_sub = self.create_subscription(
            JointState,
            'encoder_readings',
            self.encoder_readings_callback,
            QoSProfile(
                depth=50,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )
        self.camera_node_pub = self.create_subscription(
            Image,
            'camera_node',
            self.camera1_callback,
            QoSProfile(
                depth=50,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
            )
        )


    def camera1_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = cv2.resize(cv_image, (427, 240))
        self.camera_update_signal.emit(cv_image)

        

    def update_camera_view(self, cv_image):
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qt_image)
        self.ui.camera1_stream.setPixmap(pixmap)


    def encoder_readings_callback(self, msg):
        positions = {}
        velocities = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                positions[name] = np.rad2deg(msg.position[i])
            if i < len(msg.velocity):
                velocities[name] = msg.velocity[i]

        # Emit signal safely to GUI thread
        self.joint_update_signal.emit(positions, velocities)
        
    def update_gui(self, positions, velocities):
        for joint in ['joint1','joint2','joint3','joint4','joint5','joint6']:
            pos_widget = getattr(self.ui, f"{joint}_position")
            pos_widget.setText(f"{positions.get(joint, 0.0):.6f}")
            vel_widget = getattr(self.ui, f"{joint}_velocity")
            vel_widget.setText(f"{velocities.get(joint, 0.0):.6f}")


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    telemetry_node = TelemetryGuiNode(ui)
    executor = MultiThreadedExecutor()
    executor.add_node(telemetry_node)
    thread = Thread(target=executor.spin)
    thread.start()
    try:
        MainWindow.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        telemetry_node.destroy_node()
        executor.shutdown()
if __name__ == '__main__':
    main()