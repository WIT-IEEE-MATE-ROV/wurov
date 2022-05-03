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
        self.history_max_length = 3


    # Filter out noise using a rolling average.
    def rolling_avg(self, imu: Imu):

        if self.history.__len__() >= self.history_max_length:
            for point in self.history:
                imu.linear_acceleration.x +=point.linear_acceleration.x
                imu.linear_acceleration.y +=point.linear_acceleration.y
                imu.linear_acceleration.z +=point.linear_acceleration.z
                imu.angular_velocity.x +=point.angular_velocity.x
                imu.angular_velocity.y +=point.angular_velocity.y
                imu.angular_velocity.z +=point.angular_velocity.z
                imu.orientation.x +=point.orientation.x
                imu.orientation.y +=point.orientation.y
                imu.orientation.z +=point.orientation.z
                imu.orientation.w +=point.orientation.w

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

            del self.history[0]

        self.history.append(imu)


if __name__ == '__main__':
    rospy.init_node('rolling_avg', anonymous=True)
    r_a = RollingAvg('imu/data')    # Publish filtered imu to imu/data
    imu_sub = rospy.Subscriber('imu/data_raw', Imu, r_a.rolling_avg)
    rospy.spin()
