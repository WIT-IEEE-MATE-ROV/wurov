#!/usr/bin/env python3

import rospy
import numpy as np
from brping import Ping360
from sensor_msgs.msg import LaserScan


class ping360:
    def __init__(self):
        """
        ping360 node is used to get temperature and depth readings from a ping360 sensor
        duration is taken from the launch file param server
        """
        
        rospy.init_node('temperature', anonymous=True)

        self._publisher = rospy.Publisher('vehicle/ping360', LaserScan, queue_size=3)

        publishDuration = rospy.get_param(f"/{rospy.get_name()}/rate")
        interface = rospy.get_param(f"/{rospy.get_name()}/interface")
        baudrate = rospy.get_param(f"/{rospy.get_name()}/baudrate")

        self.sensor = Ping360()
        self.sensor.connect_serial(interface, baudrate)
        self.sensor.initialize()

        rospy.Timer(rospy.Duration(publishDuration), self.publisher)

        rospy.spin()

    def publisher(self, data):
        msg = LaserScan()
        LaserScan.angle_min = 0
        LaserScan.angle_max = 400
        LaserScan.angle_increment = 20

        LaserScan.ranges = []

        for val in range(20):
            LaserScan.ranges.append(self.sensor.transmitAngle(val * 20))

        self._publisher.publish(msg)

if __name__ == '__main__':
    ping360() 
