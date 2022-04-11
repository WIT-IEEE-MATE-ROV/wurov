#!/usr/bin/env python3

import math

import board;
import busio;
import adafruit_fxas21002c;
import adafruit_fxos8700;
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from tf.transformations import quaternion_from_euler


class imu_data:
    def __init__(self):
        rospy.init_node('imu_data', anonymous=True)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.gyroSensor = adafruit_fxas21002c.FXAS21002C(i2c)
        self.sensor = adafruit_fxos8700.FXOS8700(i2c)
        
        # imu_filter_madgwick
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=3)
        self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=3)

        rospy.Timer(rospy.Duration(0.1), self.imuRead)

        rospy.spin()

    def imuRead(self, data):
        imu_msg = Imu()
        mag_msg = MagneticField()

        accel_x, accel_y, accel_z = self.sensor.accelerometer                   # in m/s^2
        mag_x, mag_y, mag_z = [k/1000000 for k in self.sensor.magnetometer]     # in Tesla
        ang_x, ang_y, ang_z = self.gyroSensor.gyroscope                         # in Radians/s

        current_time = rospy.Time.now()
        imu_msg.header.stamp = current_time
        mag_msg.header.stamp = current_time
        imu_msg.header.frame_id = ''                # Need to do URUF

        # Imu msg
        pitch = 180 * math.atan2(accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))/math.pi
        roll = 180 * math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z))/math.pi
        yaw_x = mag_x*math.cos(pitch) + mag_y*math.sin(roll)*math.sin(pitch) + mag_z*math.cos(roll)*math.sin(pitch)
        yaw_y = mag_y*math.cos(roll) - mag_z*math.sin(roll)
        yaw = 180 * math.atan2(-yaw_y,yaw_x)/math.pi
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z 

        imu_msg.angular_velocity.x = ang_x
        imu_msg.angular_velocity.y = ang_y
        imu_msg.angular_velocity.z = ang_z

        # no covariance
        imu_msg.orientation_covariance[0] = -1
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.linear_acceleration_covariance[0] = -1

        # mag msg
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z

        # publish msgs
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)


if __name__ == '__main__':
    imu_data()