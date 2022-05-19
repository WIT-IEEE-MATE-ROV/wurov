#!/usr/bin/env python3

from simple_pid import PID
import rospy
from sensor_msgs.msg import Imu
from wurov.msg import trajectory
import argparse
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class simulate_imu_data:
    def __init__(self):  
        rospy.init_node('pid', anonymous=True)

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

        self.yawPID = PID(1, 0.1, 0.05)
        self.pitchPID = PID(1, 0.1, 1)
        self.xAcelPID = PID(1, 0.1, 1)
        self.yAcelPID = PID(1, 1, 1)
        self.zAcelPID = PID(1, 1, 1)

        rospy.spin()

    def updateCurrent(self, data):
        self.currentPitch = data.orientation.x
        self.currentYaw = data.orientation.y
        self.currentAccel_x = data.linear_acceleration.x
        self.currentAccel_y = data.linear_acceleration.y
        self.currentAccel_z = data.linear_acceleration.z


    def updateSetpoint(self, data):
        self.pitchPID.setpoint = data.filtered_ang_z
        self.yawPID.setpoint = data.filtered_ang_y
        self.xAcelPID.setpoint = data.filtered_accel_x
        self.yAcelPID.setpoint = data.filtered_accel_y
        self.zAcelPID.setpoint = data.filtered_accel_z

    def pid(self, data):
        msg = trajectory()

        msg.orientation.pitch = self.yawPID(self.currentYaw)
        msg.orientation.yaw = self.pitchPID(self.currentPitch)
        msg.translation.x = self.xAcelPID(self.currentAccel_x)
        msg.translation.y = self.yAcelPID(self.currentAccel_y)
        msg.translation.z = self.zAcelPID(self.currentAccel_z)

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

