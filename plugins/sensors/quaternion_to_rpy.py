#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from wurov.msg import orientation


class imu_data:
    def __init__(self):


        self.rpy = orientation()

        # imu_filter_madgwick input topics
        rospy.init_node('quaternion_to_rpy', anonymous=True)
        self.rpy_pub = rospy.Publisher('imu/rpy', orientation, queue_size=3)

        rospy.Subscriber("imu/data", Imu, self.quartenion_to_euler)

        rospy.spin()

    def quartenion_to_euler(self, data: Imu):

        self.rpy.header.stamp  = data.header.stamp 
        self.rpy.header.frame_id = data.header.frame_id

        quaternion = [
            data.orientation.x, data.orientation.y,
            data.orientation.z, data.orientation.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)

        self.rpy.roll = roll
        self.rpy.pitch = pitch
        self.rpy.yaw = yaw

        self.rpy_pub.publish(self.rpy)


if __name__ == '__main__':
    imu_data()