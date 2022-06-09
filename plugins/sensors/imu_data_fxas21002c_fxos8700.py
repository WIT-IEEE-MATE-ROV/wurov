#!/usr/bin/env python3

import board;
import busio;
import adafruit_fxas21002c;
import adafruit_fxos8700;
import rospy
from sensor_msgs.msg import Imu, MagneticField
import time


class imu_data:
    def __init__(self):
        # init hardwares
        i2c = busio.I2C(board.SCL, board.SDA)
        self.gyroSensor = adafruit_fxas21002c.FXAS21002C(i2c)
        self.sensor = adafruit_fxos8700.FXOS8700(i2c)

        # init messages
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        # init offset values
        self.linear_accel_offset = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        self.angular_vel_offset = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        self.magnetic_field_offset = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        
        # Calculate accel offset
        self.calculate_accel_offset()

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
        # REP103:
        # +x: forward
        # +y: left
        # +z: up
        accel_y, accel_z, accel_x = self.sensor.accelerometer                   # in m/s^2
        ang_y, ang_z, ang_x = self.gyroSensor.gyroscope                         # in Radians/s

        # TODO: mag needs calibrations
        mag_y, mag_z, mag_x = [k/1000000 for k in self.sensor.magnetometer]     # in Tesla


        current_time = rospy.Time.now()

        # Imu msg
        self.imu_msg.header.stamp = current_time
        self.imu_msg.header.frame_id = 'base_link'
        self.imu_msg.linear_acceleration.x = accel_x - self.linear_accel_offset['x']
        self.imu_msg.linear_acceleration.y = accel_y - self.linear_accel_offset['y']
        self.imu_msg.linear_acceleration.z = -accel_z  - self.linear_accel_offset['z']
        self.imu_msg.angular_velocity.x = ang_x - self.angular_vel_offset['x']
        self.imu_msg.angular_velocity.y = ang_y - self.angular_vel_offset['y']
        self.imu_msg.angular_velocity.z = ang_z - self.angular_vel_offset['z']

        # Mag msg
        self.mag_msg.header.stamp = current_time
        self.mag_msg.header.frame_id = 'base_link'
        self.mag_msg.magnetic_field.x = mag_x - self.magnetic_field_offset['x']
        self.mag_msg.magnetic_field.y = mag_y - self.magnetic_field_offset['y']
        self.mag_msg.magnetic_field.z = mag_z - self.magnetic_field_offset['z']

        # publish msgs
        self.imu_pub.publish(self.imu_msg)
        self.mag_pub.publish(self.mag_msg)

    def calculate_accel_offset(self, duration=2, sampling_rate=10):
        duration = duration + time.time()
        period = 1/sampling_rate
        x = []
        y = []
        z = []
        while time.time() < duration:
            accel_y, accel_z, accel_x = self.sensor.accelerometer                   # in m/s^2
            x.append(accel_x)
            y.append(accel_y)
            z.append(accel_z)
            time.sleep(period)
        self.linear_accel_offset['x'] = sum(x)/len(x) + 9.8
        self.linear_accel_offset['y'] = sum(y)/len(y)
        self.linear_accel_offset['z'] = sum(z)/len(z)
        



        
if __name__ == '__main__':
    imu_data()