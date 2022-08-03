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
from sensor_msgs.msg import Imu, MagneticField, CompressedImage

import rospkg 


# Configurations
fullscreenMode = True
# Enter Screen Resolution for surface station(Only if fullscreenMode is disabled)
height = 1080
width = 1920
# Fonts
Header = QtGui.QFont("Roboto", 12, QtGui.QFont.Bold)
Body = QtGui.QFont("Roboto", 10)


class surface_ui(QWidget):
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.update_camera)
        rospy.Subscriber('imu/data', Imu, self.update_filtered_imu)
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('wurov')

        # Style settings for GUI
        super(surface_ui, self).__init__()
        self.setGeometry(0, 0, width, height)
        self.setWindowTitle("WUROV Control")
        self.setStyleSheet('background-color: ' + '#2C2F33' + ';color: ' + 'white')

        # Grid layout for GUI
        self.widgets()
        grid = QtWidgets.QGridLayout()

        grid.addWidget(self.camera, 0,0)

        grid.addWidget(self.speed, 1, 0)
        self.setLayout(grid)


    # NineDof labels to display sensor read outs
    def widgets(self):
        #Camera widget
        self.camera = QtWidgets.QLabel(self)
        pixmap = QPixmap(f'{self.path}/surface/UI/default.jpg')
        self.camera.setPixmap(pixmap)
        #Speedometer widget
        self.speed = roundProgressBar()
        self.speed.rpb_setRange(0, 1.5) 
        self.speed.rpb_setTextFormat('Value')
        self.speed.rpb_setValue(0)

    def update_filtered_imu(self, data):
        acel_mag = math.sqrt(data.linear_acceleration.x**2 + data.linear_acceleration.y**2 + data.linear_acceleration.z**2)
        self.speed.rpb_setValue(acel_mag)

    def update_camera(self, data): #update camera based on ROS message
        qimage = QtGui.QImage.fromData(data.data)
        self.camera.setPixmap(QtGui.QPixmap.fromImage(qimage))

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
        if fullscreenMode:
            win.showFullScreen()
        else:
            win.show()
        sys.exit(app.exec_())


    window()