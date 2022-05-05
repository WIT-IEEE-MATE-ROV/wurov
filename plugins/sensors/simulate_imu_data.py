#!/usr/bin/env python3

"""

 This file is part of Enbarr.

    Enbarr is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Enbarr is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Enbarr.  If not, see <https://www.gnu.org/licenses/>.

"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField


class simulate_imu_data:
    def __init__(self):
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

         # zeros matrix for unknow covariance according to sensor_msgs/Imu doc
        zeros_mat = [0]*9
        self.imu_msg.orientation_covariance = zeros_mat
        self.imu_msg.angular_velocity_covariance = zeros_mat
        self.imu_msg.linear_acceleration_covariance = zeros_mat
        
        rospy.init_node('simulate_imu_9d', anonymous=True)

        # imu_filter_madgwick's required topics
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=3)
        self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=3)

        rospy.Timer(rospy.Duration(0.1), self.publisher)

        rospy.spin()

    def publisher(self, data):
        # While this node is still running, keep getting sensor values
        # In this case, it's simulated, so we're making up values
        # In your case, replace this block with however you're getting values
        # A real sensor is likley providing accelerometer, gyroscope, and magnetometer values
        # Combine them to produce more accure roll/pitch/yaw/x/y/z values
        current_time = rospy.Time.now()

        # Imu msg
        self.imu_msg.header.stamp = current_time
        self.imu_msg.header.frame_id = ''
        self.imu_msg.orientation.x = np.random.normal()
        self.imu_msg.orientation.y = np.random.normal()
        self.imu_msg.orientation.z = np.random.normal()
        self.imu_msg.orientation.w = np.random.normal()
        self.imu_msg.linear_acceleration.x = np.random.normal()
        self.imu_msg.linear_acceleration.y = np.random.normal()
        self.imu_msg.linear_acceleration.z = np.random.normal() 
        self.imu_msg.angular_velocity.x = np.random.normal()
        self.imu_msg.angular_velocity.y = np.random.normal()
        self.imu_msg.angular_velocity.z = np.random.normal()

        # Mag msg
        self.mag_msg.header.stamp = current_time
        self.mag_msg.header.frame_id = ''
        self.mag_msg.magnetic_field.x = np.random.normal()
        self.mag_msg.magnetic_field.y = np.random.normal()
        self.mag_msg.magnetic_field.z = np.random.normal()

        # publish msgs
        self.imu_pub.publish(self.imu_msg)
        self.mag_pub.publish(self.mag_msg)

if __name__ == '__main__':
    simulate_imu_data() 

