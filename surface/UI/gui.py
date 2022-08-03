#!/usr/bin/env python3

from PySide2 import QtCore, QtGui, QtWidgets
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication, QWidget
from PySide2.QtGui import *
from PySide2.QtWidgets import *
from PySide2.QtCore import *
import os
import sys
import rospy
import math
from PySide2extn.RoundProgressBar import roundProgressBar

from wurov.msg import system_stats
from sensor_msgs.msg import Imu, MagneticField, CompressedImage, Image

from cv2 import circle as cv2_circle
from cv_bridge import CvBridge, CvBridgeError

import rospkg


# Configurations
FULL_SCREEN = True
# Enter Screen Resolution for surface station(Only if FULL_SCREEN is disabled)
HEIGHT = 1080
WIDTH = 1920
# Fonts
HEADER_FONT = QtGui.QFont("Roboto", 12, QtGui.QFont.Bold)
BODY_FONT = QtGui.QFont("Roboto", 10)


class surface_ui(QWidget):
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        rospy.Subscriber('/usb_cam/image_raw/compressed',
                         CompressedImage, self.update_camera)
        rospy.Subscriber('imu/data', Imu, self.update_filtered_imu)
        rospy.Subscriber('/ping360_node/sonar/images',
                         Image, self.update_sonar)

        rospack = rospkg.RosPack()
        self.path = rospack.get_path('wurov')

        # for sonar
        self.cv_bridge = CvBridge()

        # Style settings for GUI
        super(surface_ui, self).__init__()
        self.setGeometry(0, 0, WIDTH, HEIGHT)
        self.setWindowTitle("WUROV Control")
        self.setStyleSheet('background-color: ' +
                           '#2C2F33' + ';color: ' + 'white')

        # Grid layout for GUI
        self.widgets()
        grid = QtWidgets.QGridLayout()

        grid.addWidget(self.camera, 0, 0)
        grid.addWidget(self.speed, 1, 0)
        grid.addWidget(self.sonar, 0, 1)

        self.setLayout(grid)

    # NineDof labels to display sensor read outs

    def widgets(self):
        # Camera widget
        self.camera = QtWidgets.QLabel(self)
        pixmap = QPixmap(f'{self.path}/surface/UI/default.jpg')
        self.camera.setPixmap(pixmap)
        # Speedometer widget
        self.speed = roundProgressBar()
        self.speed.rpb_setRange(0, 1.5)
        self.speed.rpb_setTextFormat('Value')
        self.speed.rpb_setValue(0)
        # Sonar widget
        self.sonar = QtWidgets.QLabel(self)
        pixmap = QPixmap(f'{self.path}/surface/UI/default.jpg')
        self.sonar.setPixmap(pixmap)

    def update_filtered_imu(self, data):
        acel_mag = math.sqrt(data.linear_acceleration.x**2 +
                             data.linear_acceleration.y**2 + data.linear_acceleration.z**2)
        self.speed.rpb_setValue(acel_mag)

    def update_camera(self, data):  # update camera based on ROS message
        qimage = QtGui.QImage.fromData(data.data)
        self.camera.setPixmap(QtGui.QPixmap.fromImage(qimage))

    def update_sonar(self, data):  # update sonar based on ROS message
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data.data, data.encoding)
        except CvBridgeError as e:
            print(e)
        (rows, cols) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2_circle(cv_image, (50, 50), 10, 255)
        qimage = QtGui.QImage.fromData(cv_image.data)
        # qimage = QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()

        self.sonar.setPixmap(QtGui.QPixmap.fromImage(qimage))

    def alert(self, s):
        """
        This handle errors and displaying alerts.
        """
        err = QErrorMessage(self)
        err.showMessage(s)


# Code to be executed
if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    win = surface_ui()

    def window():
        if FULL_SCREEN:
            win.showFullScreen()
        else:
            win.show()
        sys.exit(app.exec_())

    window()
