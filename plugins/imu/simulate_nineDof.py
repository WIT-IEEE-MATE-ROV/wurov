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
from wurov.msg import ninedof


class simulate_nineDof:
    def __init__(self):
        rospy.init_node('simulate_nineDof', anonymous=True)

        self.nineDof_current_pub = rospy.Publisher(
            'ninedof_values', ninedof, queue_size=3)

        rospy.Timer(rospy.Duration(0.1), self.publisher)

        rospy.spin()

    def publisher(self, data):
        sendval_ninedof = ninedof()
        # While this node is still running, keep getting sensor values
        # In this case, it's simulated, so we're making up values
        # In your case, replace this block with however you're getting values
        # A real sensor is likley providing accelerometer, gyroscope, and magnetometer values
        # Combine them to produce more accure roll/pitch/yaw/x/y/z values
        sendval_ninedof.orientation.roll = np.random.normal()
        sendval_ninedof.orientation.pitch = np.random.normal()
        sendval_ninedof.orientation.yaw = np.random.normal()
        sendval_ninedof.translation.x = np.random.normal()
        sendval_ninedof.translation.y = np.random.normal()
        sendval_ninedof.translation.z = np.random.normal()
        sendval_ninedof.magnetometer.x = np.random.normal()
        sendval_ninedof.magnetometer.y = np.random.normal()
        sendval_ninedof.magnetometer.z = np.random.normal()

        self.nineDof_current_pub.publish(sendval_ninedof)


if __name__ == '__main__':
    simulate_nineDof()
