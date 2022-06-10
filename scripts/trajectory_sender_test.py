#!/usr/bin/env python

import rospy
from auv.msg import trajectory


def publisher():
    pub = rospy.Publisher('trajectory_commands', trajectory, queue_size=3)
    rospy.init_node('trajectory_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        msg = trajectory()
        msg.angular.x = .1
        msg.angular.y = -.2
        msg.angular.z = .3
        msg.linear.x = -.4
        msg.linear.y = .5
        msg.linear.z = -.6

        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
