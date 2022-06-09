#!/usr/bin/env python3

import rospy
import numpy as np
import ms5837
from std_msgs.msg import Float32


class bar30:
    def __init__(self):
        """
        bar30 node is used to get temperature and depth readings from a bar30 sensor
        duration is taken from the launch file param server
        """
        
        rospy.init_node('temperature', anonymous=True)

        self.depth_publisher = rospy.Publisher('vehicle/temperature', Float32, queue_size=3)
        self.temp_publisher = rospy.Publisher('vehicle/depth', Float32, queue_size=3)

        publishDuration = rospy.get_param(f"/{rospy.get_name()}/rate")

        self.sensor = ms5837.MS5837_30BA()
        self.sensor.init()

        try:
            fluidDensity = rospy.get_param(f"/{rospy.get_name()}/fluidDensity")
            self.sensor.setFluidDensity(fluidDensity)
        except Exception as e:
            rospy.loginfo("No fluid density specified, using default")

        rospy.Timer(rospy.Duration(publishDuration), self.publisher)

        rospy.spin()

    def publisher(self, data):
        self.sensor.read()

        self.temp_publisher.publish(self.sensor.depth())
        self.depth_publisher.publish(self.sensor.temperature())

if __name__ == '__main__':
    bar30() 
