#!/usr/bin/env python3

from simple_pid import PID
import rospy
from sensor_msgs.msg import Imu
from wurov.msg import trajectory
import argparse
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
import time

class simulate_imu_data:
    def __init__(self):  
        rospy.init_node('pid', anonymous=True)
        time.sleep(15) #sleep for init cycle
        parser = argparse.ArgumentParser("Dynamic Reconfig")
        parser.add_argument('--dynamic_reconfig', type=bool, help='set to true if dynamic reconfig should be enabled')
        self.args = parser.parse_args(rospy.myargv()[1:])

        if self.args.dynamic_reconfig:
            self.dynamic_rec()

        #set initial current values to 0
        self.currentPitch = 0
        self.currentYaw = 0
        self.currentAccel_x = 0
        self.currentAccel_y = 0
        self.currentAccel_z = 0

        rospy.Subscriber('imu/data', Imu, self.updateCurrent)
        rospy.Subscriber('trajectory_request', trajectory, self.updateSetpoint)

        self._publisher = rospy.Publisher('trajectory_corrected', trajectory, queue_size=3)

        rospy.Timer(rospy.Duration(0.1), self.pid)

        self.yawPID = PID(0.3, 0, 0)
        self.pitchPID = PID(0.3, 0, 0)
        self.xAcelPID = PID(0.3, 0, 0)
        self.yAcelPID = PID(0.3, 0, 0)
        self.zAcelPID = PID(0.3, 0, 0)

        #Set initial trajectory_request to 0
        self.yaw_value = 0
        self.pitch_value = 0
        self.xAcelPID_value = 0
        self.yAcelPID_value = 0
        self.zAcelPID_value = 0

        rospy.spin()

    def updateCurrent(self, data):
        self.currentPitch = data.angular_velocity.y
        self.currentYaw = data.angular_velocity.z
        self.currentAccel_x = data.linear_acceleration.x
        self.currentAccel_y = data.linear_acceleration.y
        self.currentAccel_z = data.linear_acceleration.z


    def updateSetpoint(self, data):
        self.pitchPID.setpoint = data.angular.y
        self.yawPID.setpoint = data.angular.z
        self.xAcelPID.setpoint = data.linear.x
        self.yAcelPID.setpoint = data.linear.y
        self.zAcelPID.setpoint = data.linear.z

    def pid(self, data):
        #TODO: Replace with geometry_msg
        msg = trajectory()

        #add step
        self.yaw_value += self.yawPID(self.currentYaw)
        self.pitch_value += self.pitchPID(self.currentYaw)
        self.xAcelPID_value += self.xAcelPID(self.currentYaw)
        self.yAcelPID_value += self.yAcelPID(self.currentYaw)
        self.zAcelPID_value += self.zAcelPID(self.currentYaw)

        msg.header.stamp = rospy.Time.now()
        msg.angular.y = self.yaw_value
        msg.angular.z = self.pitch_value
        msg.linear.x = self.xAcelPID_value
        msg.linear.y = self.yAcelPID_value
        msg.linear.z = self.zAcelPID_value

        self._publisher.publish(msg)

    def dynamic_rec(self):
        self.ddynrec = DDynamicReconfigure("pid_rec")

        self.ddynrec.add_variable("yawP", "yaw proportion scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("yawI", "yaw integral scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("yawD", "yaw derrivative scale", 0.0, -1.0, 1.0)

        self.ddynrec.add_variable("rollP", "rollP proportion scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("rollI", "rollP integral scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("rollD", "rollP derrivative scale", 0.0, -1.0, 1.0)

        self.ddynrec.add_variable("xAcelP", "xAcelP proportion scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("xAcelI", "xAcelP integral scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("xAcelD", "xAcelP derrivative scale", 0.0, -1.0, 1.0)

        self.ddynrec.add_variable("yAcelP", "yAcelP proportion scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("yAcelI", "yAcelP integral scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("yAcelD", "yAcelP derrivative scale", 0.0, -1.0, 1.0)

        self.ddynrec.add_variable("zAcelP", "zAcelP proportion scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("zAcelI", "zAcelP integral scale", 0.0, -1.0, 1.0)
        self.ddynrec.add_variable("zAcelD", "zAcelP derrivative scale", 0.0, -1.0, 1.0)

        self.ddynrec.start(self.dyn_rec_callback)

    def dyn_rec_callback(self, config, data):

        self.yawPID = PID(config["yawP"], config["yawI"], config["yawD"])
        self.pitchPID = PID(config["rollP"], config["rollI"], config["rollD"])
        self.xAcelPID = PID(config["xAcelP"], config["xAcelI"], config["xAcelD"])
        self.yAcelPID = PID(config["yAcelP"], config["yAcelI"], config["yAcelD"])
        self.zAcelPID = PID(config["zAcelP"], config["zAcelI"], config["zAcelD"])

        return config

if __name__ == '__main__':
    simulate_imu_data() 

