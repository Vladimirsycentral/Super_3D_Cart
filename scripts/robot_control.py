
#!/usr/bin/python3
import math
import threading
import sys
import rclpy
import cv2
from cv_bridge import CvBridge
import numpy as np
import PyQt5
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Color
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDateEdit,
    QDateTimeEdit,
    QDial,
    QDoubleSpinBox,
    QFontComboBox,
    QLabel,
    QHBoxLayout,
    QLCDNumber,
    QLineEdit,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QSlider,
    QSpinBox,
    QTimeEdit,
    QVBoxLayout,
    QWidget,
)


vel = 0.0


class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        #self.pub_vel = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)
        self.camera_sub = self.create_subscription(Image, '/rear_camera/image_raw', self.sensor_callback, 10)
        self.camera_sub_front = self.create_subscription(Image, '/front_camera/image_raw', self.front_camera_callback, 10)
        self.create_subscription(LaserScan, '/lf/lidar', self.lf_sensor_callback, 10)
        self.create_subscription(LaserScan, '/ls/lidar', self.ls_sensor_callback, 10)
        self.create_subscription(LaserScan, '/rf/lidar', self.rf_sensor_callback, 10)
        self.create_subscription(LaserScan, '/rs/lidar', self.rs_sensor_callback, 10)
        self.create_subscription(LaserScan, '/rl/lidar', self.lr_sensor_callback, 10)
        self.create_subscription(LaserScan, '/rr/lidar', self.rr_sensor_callback, 10)

        #self.timer = self.create_timer(1, self.timer_callback)
        self.bridge = CvBridge()

    def lf_sensor_callback(self, msg):
        global sensor_label
        lst = ['{:.2f}'.format(x) for x in msg.ranges]
        total = '['
        for x in lst:
            total += x + ' '
        total += ']'
        print(total)
        sensor_label.setText(total)

    def ls_sensor_callback(self, msg):
        global sensor_label_2
        lst = ['{:.2f}'.format(x) for x in msg.ranges]
        total = '['
        for x in lst:
            total += x + ' '
        total += ']'
        sensor_label_2.setText(total)
    
    def rf_sensor_callback(self, msg):
        global sensor_label_3
        lst = ['{:.2f}'.format(x) for x in msg.ranges]
        total = '['
        for x in lst:
            total += x + ' '
        total += ']'
        sensor_label_3.setText(total)

    def rs_sensor_callback(self, msg):
        global sensor_label_4
        lst = ['{:.2f}'.format(x) for x in msg.ranges]
        total = '['
        for x in lst:
            total += x + ' '
        total += ']'
        sensor_label_4.setText(total)

    def lr_sensor_callback(self, msg):
        global sensor_label_5
        lst = ['{:.2f}'.format(x) for x in msg.ranges]
        total = '['
        for x in lst:
            total += x + ' '
        total += ']'
        sensor_label_5.setText(total)

    def rr_sensor_callback(self, msg):
        global sensor_label_6
        lst = ['{:.2f}'.format(x) for x in msg.ranges]
        total = '['
        for x in lst:
            total += x + ' '
        total += ']'
        sensor_label_6.setText(total)


    def sensor_callback(self, msg):
        global stream_label
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, c = img.shape
        b = 3 * w
        img = QImage(img.data, w, h, b, QImage.Format_RGB888).rgbSwapped()
        stream_label.setPixmap(QPixmap.fromImage(img))


    def front_camera_callback(self, msg):
        global stream_label_2
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, c = img.shape
        b = 3 * w
        img = QImage(img.data, w, h, b, QImage.Format_RGB888).rgbSwapped()
        stream_label_2.setPixmap(QPixmap.fromImage(img))


    def timer_callback(self):
        global vel
        print(vel)
        msg = Twist()
        msg.linear.x = vel
        self.pub_vel.publish(msg)



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        global stream_label, stream_label_2, sensor_label, sensor_label_2, sensor_label_3, sensor_label_4, sensor_label_5, sensor_label_6
        self.setWindowTitle("Widgets App")
        hlayout = QHBoxLayout()
        stream_label = QLabel()
        stream_label_2 = QLabel()
        sensor_label = QLabel()
        sensor_label_2 = QLabel()
        sensor_label_3 = QLabel()
        sensor_label_4 = QLabel()
        sensor_label_5 = QLabel()
        sensor_label_6 = QLabel()
        layout = QVBoxLayout()
        layout_2 = QVBoxLayout()
        layout.addWidget(stream_label)
        layout.addWidget(stream_label_2)
        layout_2.addWidget(sensor_label)
        layout_2.addWidget(sensor_label_2)
        layout_2.addWidget(sensor_label_3)
        layout_2.addWidget(sensor_label_4)
        layout_2.addWidget(sensor_label_5)
        layout_2.addWidget(sensor_label_6)
        hlayout.addLayout(layout)
        hlayout.addLayout(layout_2)
        
        widget = QWidget()
        widget.setLayout(hlayout)
        self.setCentralWidget(widget)
     
     
if __name__ == '__main__':
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    window = MainWindow()
    
    commander = Commander()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    window.show()

    app.exec()
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()

