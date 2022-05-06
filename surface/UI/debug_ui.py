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
from sensor_msgs.msg import Imu, MagneticField

# Configurations
fullscreenMode = True
cameraIndex1 = 2
cameraIndex2 = 1
# Enter Screen Resolution for surface station(Only if fullscreenMode is disabled)
height = 1080
width = 1920
# Fonts
Header = QtGui.QFont("Roboto", 12, QtGui.QFont.Bold)
Body = QtGui.QFont("Roboto", 10)


class WUROV(QWidget):
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        rospy.Subscriber('imu/data_raw', Imu, self.update_raw_imu)
        rospy.Subscriber('imu/data', Imu, self.update_filtered_imu)

        # Style settings for GUI
        super(WUROV, self).__init__()
        self.setGeometry(0, 0, width, height)
        self.setWindowTitle("WUROV Control")
        self.setStyleSheet('background-color: ' +
                           '#2C2F33' + ';color: ' + 'white')

        # Grid layout for GUI
        self.nineDof()
        grid = QtWidgets.QGridLayout()

        grid.addWidget(self.raw_title, 1, 0)
        grid.addWidget(self.raw_accel_x, 2, 0)
        grid.addWidget(self.raw_accel_y, 3, 0)
        grid.addWidget(self.raw_accel_z, 4, 0)
        grid.addWidget(self.raw_ang_x, 5, 0)
        grid.addWidget(self.raw_ang_y, 6, 0)
        grid.addWidget(self.raw_ang_z, 7, 0)

        grid.addWidget(self.filtered_title, 1, 1)
        grid.addWidget(self.filtered_accel_x, 2, 1)
        grid.addWidget(self.filtered_accel_y, 3, 1)
        grid.addWidget(self.filtered_accel_z, 4, 1)
        grid.addWidget(self.filtered_ang_x, 5, 1)
        grid.addWidget(self.filtered_ang_y, 6, 1)
        grid.addWidget(self.filtered_ang_z, 7, 1)

        grid.addWidget(self.speed_title, 1, 2)
        grid.addWidget(self.speed, 2, 2)
        self.setLayout(grid)

    # NineDof labels to display sensor read outs

    def nineDof(self):
        # raw ninedof
        self.raw_title = QtWidgets.QLabel(self)
        self.raw_title.setFont(QtGui.QFont(Header))
        self.raw_title.adjustSize()
        self.raw_title.setText("Raw IMU: ")

        self.raw_accel_x = QtWidgets.QLabel(self)
        self.raw_accel_x.setFont(QtGui.QFont(Body))
        self.raw_accel_x.adjustSize()

        self.raw_accel_y = QtWidgets.QLabel(self)
        self.raw_accel_y.setFont(QtGui.QFont(Body))
        self.raw_accel_y.adjustSize()

        self.raw_accel_z = QtWidgets.QLabel(self)
        self.raw_accel_z.setFont(QtGui.QFont(Body))
        self.raw_accel_z.adjustSize()

        self.raw_ang_x = QtWidgets.QLabel(self)
        self.raw_ang_x.setFont(QtGui.QFont(Body))
        self.raw_ang_x.adjustSize()

        self.raw_ang_y = QtWidgets.QLabel(self)
        self.raw_ang_y.setFont(QtGui.QFont(Body))
        self.raw_ang_y.adjustSize()

        self.raw_ang_z = QtWidgets.QLabel(self)
        self.raw_ang_z.setFont(QtGui.QFont(Body))
        self.raw_ang_z.adjustSize()
        # filtered ninedof
        self.filtered_title = QtWidgets.QLabel(self)
        self.filtered_title.setFont(QtGui.QFont(Header))
        self.filtered_title.adjustSize()
        self.filtered_title.setText("Filtered IMU: ")

        self.filtered_accel_x = QtWidgets.QLabel(self)
        self.filtered_accel_x.setFont(QtGui.QFont(Body))
        self.filtered_accel_x.adjustSize()

        self.filtered_accel_y = QtWidgets.QLabel(self)
        self.filtered_accel_y.setFont(QtGui.QFont(Body))
        self.filtered_accel_y.adjustSize()

        self.filtered_accel_z = QtWidgets.QLabel(self)
        self.filtered_accel_z.setFont(QtGui.QFont(Body))
        self.filtered_accel_z.adjustSize()

        self.filtered_ang_x = QtWidgets.QLabel(self)
        self.filtered_ang_x.setFont(QtGui.QFont(Body))
        self.filtered_ang_x.adjustSize()

        self.filtered_ang_y = QtWidgets.QLabel(self)
        self.filtered_ang_y.setFont(QtGui.QFont(Body))
        self.filtered_ang_y.adjustSize()

        self.filtered_ang_z = QtWidgets.QLabel(self)
        self.filtered_ang_z.setFont(QtGui.QFont(Body))
        self.filtered_ang_z.adjustSize()
        # speedometer
        self.speed_title = QtWidgets.QLabel(self)
        self.speed_title.setFont(QtGui.QFont(Header))
        self.speed_title.adjustSize()
        self.speed_title.setText("Speed Data: ")

        self.speed = roundProgressBar()
        self.speed.rpb_setRange(0, 1.5)
        self.speed.rpb_setTextFormat('Value')

    # Update functions change labels based on callback
    def update_raw_imu(self, data):
        self.raw_accel_x.setText(f"Accel X: {data.linear_acceleration.x}")
        self.raw_accel_y.setText(f"Accel Y: {data.linear_acceleration.y}")
        self.raw_accel_z.setText(f"Accel Z: {data.linear_acceleration.z}")
        self.raw_ang_x.setText(f"Angular X: {data.angular_velocity.x}")
        self.raw_ang_y.setText(f"Angular Y: {data.angular_velocity.y}")
        self.raw_ang_z.setText(f"Angular Z: {data.angular_velocity.z}")

    def update_filtered_imu(self, data):
        self.filtered_accel_x.setText(f"Accel X: {data.linear_acceleration.x}")
        self.filtered_accel_y.setText(f"Accel Y: {data.linear_acceleration.y}")
        self.filtered_accel_z.setText(f"Accel Z: {data.linear_acceleration.z}")
        self.filtered_ang_x.setText(f"Angular X: {data.angular_velocity.x}")
        self.filtered_ang_y.setText(f"Angular Y: {data.angular_velocity.y}")
        self.filtered_ang_z.setText(f"Angular Z: {data.angular_velocity.z}")

        acel_mag = math.sqrt(data.linear_acceleration.x**2 +
                             data.linear_acceleration.y**2 + data.linear_acceleration.z**2)
        self.speed.rpb_setValue(acel_mag)

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
    win = WUROV()

    def window():
        if fullscreenMode:
            win.showFullScreen()
        else:
            win.show()
        sys.exit(app.exec_())

    window()
