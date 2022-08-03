#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Accel
import argparse
from std_msgs.msg import Bool
import time 

class pid_sim:
    def __init__(self):  
        rospy.init_node('pid_sim', anonymous=True)
        self.imu_publisher = rospy.Publisher('test', Imu, queue_size=3)
        self.traj_pub = rospy.Publisher('trajectory_request', Accel, queue_size=3)

        msg = Imu()
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0

        self.imu_publisher.publish(msg)
        print(msg)

        self.sim()

        rospy.spin()

    def sim(self):
        time.sleep(15)
        traj_msg = Accel()
        traj_msg.linear.x = 0.5
        traj_msg.linear.y = 0.0
        traj_msg.linear.z = 0.0
        traj_msg.angular.x = 0.0
        traj_msg.angular.y = 0.0
        traj_msg.angular.z = 0.0
        self.traj_pub.publish(traj_msg)

        msg = Imu()
        count = 0.0
        run = True
        while count < 0.45:
            msg.angular_velocity.x = count
            msg.angular_velocity.y = count
            msg.angular_velocity.z = count
            msg.linear_acceleration.x = count
            msg.linear_acceleration.y = count
            msg.linear_acceleration.z = count

            count += 0.05
            self.imu_publisher.publish(msg)

            time.sleep(0.3)
        while True:
            msg.angular_velocity.x = count
            msg.angular_velocity.y = count
            msg.angular_velocity.z = count
            msg.linear_acceleration.x = count
            msg.linear_acceleration.y = count
            msg.linear_acceleration.z = count

            self.imu_publisher.publish(msg)
            time.sleep(0.3)

if __name__ == '__main__':
    pid_sim() 

