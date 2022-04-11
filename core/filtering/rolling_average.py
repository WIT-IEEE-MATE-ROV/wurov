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
from sensor_msgs.msg import Imu


class RollingAvg:
    def __init__(self, pub_topic) -> None:
        self.imu_pub = rospy.Publisher(pub_topic, Imu, queue_size=3)
        self.history = []
        self.history_max_length = 10


    # Filter out noise using a rolling average.
    def rolling_avg(self, imu: Imu):
        self.history.append(imu)

        if self.history.__len__() > self.history_max_length:
            del self.history[0]

            imu.linear_acceleration.x = sum([point.linear_acceleration.x for point in self.history])
            imu.linear_acceleration.y = sum([point.linear_acceleration.y for point in self.history])
            imu.linear_acceleration.z = sum([point.linear_acceleration.z for point in self.history])
            imu.angular_velocity.x = sum([point.angular_velocity.x for point in self.history])
            imu.angular_velocity.y = sum([point.angular_velocity.y for point in self.history])
            imu.angular_velocity.z = sum([point.angular_velocity.z for point in self.history])
            imu.orientation.x = sum([point.orientation.x for point in self.history])
            imu.orientation.y = sum([point.orientation.y for point in self.history])
            imu.orientation.z = sum([point.orientation.z for point in self.history])
            imu.orientation.w = sum([point.orientation.w for point in self.history])


            imu.linear_acceleration.x /= self.history.__len__()
            imu.linear_acceleration.y /= self.history.__len__()
            imu.linear_acceleration.z /= self.history.__len__()
            imu.angular_velocity.x /= self.history.__len__()
            imu.angular_velocity.y /= self.history.__len__()
            imu.angular_velocity.z /= self.history.__len__()
            imu.orientation.x /= self.history.__len__()
            imu.orientation.y /= self.history.__len__()
            imu.orientation.z /= self.history.__len__()
            imu.orientation.w /= self.history.__len__()

            self.imu_pub.publish(imu)


if __name__ == '__main__':
    rospy.init_node('filter', anonymous=True)
    r_a = RollingAvg('imu/data')    # Publish to imu/data after filtering
    imu_sub = rospy.Subscriber('imu/data_raw', Imu, r_a.rolling_avg)

    rospy.spin()
