#!/usr/bin/env python3

import board;
import busio;
import adafruit_fxas21002c;
import adafruit_fxos8700;
import rospy
from sensor_msgs.msg import Imu, MagneticField


class imu_data:
    def __init__(self):
        # init hardwares
        i2c = busio.I2C(board.SCL, board.SDA)
        self.gyroSensor = adafruit_fxas21002c.FXAS21002C(i2c)
        self.sensor = adafruit_fxos8700.FXOS8700(i2c)

        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        # zeros matrix for unknow covariance according to sensor_msgs/Imu doc
        zeros_mat = [0]*9
        self.imu_msg.orientation_covariance = zeros_mat
        self.imu_msg.angular_velocity_covariance = zeros_mat
        self.imu_msg.linear_acceleration_covariance = zeros_mat

        # imu_filter_madgwick input topics
        rospy.init_node('imu_raw_data', anonymous=True)
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=3)
        self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=3)

        rospy.Timer(rospy.Duration(0.1), self.read_imu)

        rospy.spin()

    def read_imu(self, data):
        accel_x, accel_y, accel_z = self.sensor.accelerometer                   # in m/s^2
        mag_x, mag_y, mag_z = [k/1000000 for k in self.sensor.magnetometer]     # in Tesla
        ang_x, ang_y, ang_z = self.gyroSensor.gyroscope                         # in Radians/s

        current_time = rospy.Time.now()

        # Imu msg
        self.imu_msg.header.stamp = current_time
        self.imu_msg.header.frame_id = ''                # Need to do URDF
        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z 
        self.imu_msg.angular_velocity.x = ang_x
        self.imu_msg.angular_velocity.y = ang_y
        self.imu_msg.angular_velocity.z = ang_z

        # Mag msg
        self.mag_msg.header.stamp = current_time
        self.mag_msg.header.frame_id = ''                # Need to do URDF
        self.mag_msg.magnetic_field.x = mag_x
        self.mag_msg.magnetic_field.y = mag_y
        self.mag_msg.magnetic_field.z = mag_z

        # publish msgs
        self.imu_pub.publish(self.imu_msg)
        self.mag_pub.publish(self.mag_msg)


if __name__ == '__main__':
    imu_data()