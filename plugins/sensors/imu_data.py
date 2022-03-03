#!/usr/bin/env python3

import board;
import busio;
import adafruit_fxas21002c;
import adafruit_fxos8700;
import rospy
from wurov.msg import ninedof



class imu_data:
    def __init__(self):
        rospy.init_node('imu_data', anonymous=True)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.gyroSensor = adafruit_fxas21002c.FXAS21002C(i2c)
        self.sensor = adafruit_fxos8700.FXOS8700(i2c)

        self.nineDof_current_pub = rospy.Publisher('ninedof_values', ninedof, queue_size=3)

        rospy.Timer(rospy.Duration(0.1), self.imuRead)

        rospy.spin()

    def imuRead(self, data):
        sendval_ninedof = ninedof()

        sendval_ninedof.orientation.roll = self.gyroSensor.gyroscope[0]
        sendval_ninedof.orientation.pitch = self.gyroSensor.gyroscope[1]
        sendval_ninedof.orientation.yaw = self.gyroSensor.gyroscope[2]
        sendval_ninedof.translation.x = self.sensor.accelerometer[0]
        sendval_ninedof.translation.y = self.sensor.accelerometer[1]
        sendval_ninedof.translation.z = self.sensor.accelerometer[2]
        sendval_ninedof.magnetometer.x = self.sensor.magnetometer[0]
        sendval_ninedof.magnetometer.y = self.sensor.magnetometer[1]
        sendval_ninedof.magnetometer.z = self.sensor.magnetometer[2]

        self.nineDof_current_pub.publish(sendval_ninedof)

if __name__ == '__main__':
    imu_data()