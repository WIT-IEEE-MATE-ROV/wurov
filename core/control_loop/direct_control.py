#!/usr/bin/env python3

#used to bypass PID

import rospy
from geometry_msgs.msg import Accel


class direct_control:
    def __init__(self):  
        rospy.init_node('pid', anonymous=True)

        rospy.Subscriber('trajectory_request', Accel, self.publisher)

        self._publisher = rospy.Publisher('trajectory_corrected', Accel, queue_size=3)

        rospy.spin()

    def publisher(self, data):
        msg = Accel()

        msg.angular.y = data.angular.y
        msg.angular.z = data.angular.z
        msg.linear.x= data.linear.x
        msg.linear.y = data.linear.y
        msg.linear.z = data.linear.z

        self._publisher.publish(msg)


if __name__ == '__main__':
    direct_control() 

