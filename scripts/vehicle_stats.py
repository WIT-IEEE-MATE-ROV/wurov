#!/usr/bin/env python3

import rospy
from wurov.msg import system_stats
import psutil
from gpiozero import CPUTemperature

class vehicle_stats:
    def __init__(self):
        rospy.init_node('vehicle_stats', anonymous=True)

        self._publisher = rospy.Publisher(
            'vehicle_stats', system_stats, queue_size=3)

        rospy.Timer(rospy.Duration((1/4)), self.publisher)

        rospy.spin()

    def publisher(self, event):
        del event
        ret_msg = system_stats()
        ret_msg.cpu_usage = psutil.cpu_percent()

        ret_msg.temp = CPUTemperature().temperature

        self.publisher.publish(ret_msg)

if __name__ == '__main__':
    vehicle_stats()
