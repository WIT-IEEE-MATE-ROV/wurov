#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import time
from pympler.asizeof import asizeof

class latency:
    def __init__(self):
        self.run = False
        
        rospy.init_node('latency', anonymous=True)

        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.update_latency)
        self.counter = 0
        self.latency = 0
        self.prevSent = 0
        self.packageSize = 0
        self.prevSent
        rospy.spin()

    def update_latency(self, data):
        
        if (rospy.get_rostime().nsecs - data.header.stamp.nsecs) * 1e-9 > 0.0:
            self.latency += (rospy.get_rostime().nsecs - data.header.stamp.nsecs) * 1e-9
            self.counter += 1
            self.packageSize += asizeof(data)
        if self.counter % 50 == 0:
            print(f"\nTotal number of images recieved: {self.counter}")
            print(f"\nAverage Latency: {self.latency / self.counter}")
            print(f"Average size of images: {self.packageSize / self.counter}")

if __name__ == '__main__':
    latency() 
