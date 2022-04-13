#!/usr/bin/env python3

import math
import board;
import busio;
import adafruit_fxas21002c;
import adafruit_fxos8700;
import rospy
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_from_euler


class imu_data:
    def __init__(self):
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        # zeros matrix for unknow covariance according to sensor_msgs/Imu doc
        zeros_mat = [0]*9
        self.imu_msg.orientation_covariance = zeros_mat
        self.imu_msg.angular_velocity_covariance = zeros_mat
        self.imu_msg.linear_acceleration_covariance = zeros_mat

        # init hardwares
        i2c = busio.I2C(board.SCL, board.SDA)
        self.gyroSensor = adafruit_fxas21002c.FXAS21002C(i2c)
        self.sensor = adafruit_fxos8700.FXOS8700(i2c)

        rospy.init_node('imu_data', anonymous=True)

        # imu_filter_madgwick input topics
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=3)
        self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=3)

        rospy.Timer(rospy.Duration(0.1), self.read_imu)

        rospy.spin()

    def calculate_rpy(self, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z):
        pitch = 180 * math.atan2(accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))/math.pi
        roll = 180 * math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z))/math.pi
        yaw_x = mag_x*math.cos(pitch) + mag_y*math.sin(roll)*math.sin(pitch) + mag_z*math.cos(roll)*math.sin(pitch)
        yaw_y = mag_y*math.cos(roll) - mag_z*math.sin(roll)
        yaw = 180 * math.atan2(-yaw_y,yaw_x)/math.pi
        return roll, pitch, yaw

    def read_imu(self, data):
        accel_x, accel_y, accel_z = self.sensor.accelerometer                   # in m/s^2
        mag_x, mag_y, mag_z = [k/1000000 for k in self.sensor.magnetometer]     # in Tesla
        ang_x, ang_y, ang_z = self.gyroSensor.gyroscope                         # in Radians/s

        current_time = rospy.Time.now()
        self.imu_msg.header.stamp = current_time
        self.mag_msg.header.stamp = current_time
        self.imu_msg.header.frame_id = ''                # Need to do URDF

        # imu_madwick does not need quanternion
        # roll, pitch, yaw = self.calculate_rpy(accel_x, accel_y, accel_z, mag_x, mag_y, mag_z)
        # quaternion = quaternion_from_euler(roll, pitch, yaw)
        # self.imu_msg.orientation.w = quaternion[0]
        # self.imu_msg.orientation.x = quaternion[1]
        # self.imu_msg.orientation.y = quaternion[2]
        # self.imu_msg.orientation.z = quaternion[3]

        # Imu msg
        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z 
        self.imu_msg.angular_velocity.x = ang_x
        self.imu_msg.angular_velocity.y = ang_y
        self.imu_msg.angular_velocity.z = ang_z

        # Mag msg
        self.mag_msg.magnetic_field.x = mag_x
        self.mag_msg.magnetic_field.y = mag_y
        self.mag_msg.magnetic_field.z = mag_z

        # publish msgs
        self.imu_pub.publish(self.imu_msg)
        self.mag_pub.publish(self.mag_msg)


if __name__ == '__main__':
    imu_data()