#!/usr/bin/env python3

import rospy
from scripts.trajectory_sender_test import publisher
from wurov.msg import system_stats
import psutil
from gpiozero import CPUTemperature

class vehicle_stats:
    def __init__(self):
        rospy.init_node('vehicle_stats', anonymous=True)

        self.nineDof_current_pub = rospy.Publisher(
            'vehicle_stats', system_stats, queue_size=3)


        rospy.Timer(rospy.Duration(4, self.publisher)

        rospy.spin()

    def publisher(self, data):
        ret_msg = system_stats()
        ret_msg.cpu_usage = psutil.cpu_percent()
        
        cpu = CPUTemperature()

        ret_msg.temp = cpu.temperature

        self.publisher.publish(ret_msg)
if __name__ == '__main__':
    vehicle_stats()
