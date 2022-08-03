#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Accel
import argparse
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from std_msgs.msg import Bool
from pid import PID

class pid:
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
        rospy.Subscriber('trajectory_request', Accel, self.updateSetpoint)

        self._publisher = rospy.Publisher('trajectory_corrected', Accel, queue_size=3)

        rospy.Timer(rospy.Duration(0.05), self.pid)

        rospy.spin()

    def updateCurrent(self, data):
        try: #catch if pid is initialized
            self.currentPitch = data.angular_velocity.y
            self.currentYaw = data.angular_velocity.z
            self.currentAccel_x = data.linear_acceleration.x
            self.currentAccel_y = data.linear_acceleration.y
            self.currentAccel_z = data.linear_acceleration.z
        except:
            pass


    def updateSetpoint(self, data):
        try: #catch if pid is initialized
            self.pitchPID.SetPoint = data.angular.y
            self.yawPID.SetPoint = data.angular.z
            self.xAcelPID.SetPoint = data.linear.x
            self.yAcelPID.SetPoint = data.linear.y
            self.zAcelPID.SetPoint = data.linear.z
        except:
            pass

    def pid(self, data):
        #TODO: Replace with geometry_msg
        msg = Accel()

        #add step
        self.yawPID.update(self.currentYaw)
        self.pitchPID.update(self.currentYaw)
        self.xAcelPID.update(self.currentYaw)
        self.yAcelPID.update(self.currentYaw)
        self.zAcelPID.update(self.currentYaw)

        msg.angular.y = self.yawPID.output
        msg.angular.z = self.pitchPID.output
        msg.linear.x = self.xAcelPID.output 
        msg.linear.y = self.yAcelPID.output 
        msg.linear.z = self.zAcelPID.output 

        self._publisher.publish(msg)

    def dynamic_rec(self):
        self.ddynrec = DDynamicReconfigure("pid_rec")

        self.ddynrec.add_variable("yawP", "yaw proportion scale", 0.0, -1.0, 100.0)
        self.ddynrec.add_variable("yawI", "yaw integral scale", 0.0, -1.0, 100.0)
        self.ddynrec.add_variable("yawD", "yaw derrivative scale", 0.0, -1.0, 100.0)

        self.ddynrec.add_variable("rollP", "rollP proportion scale", 0.0, -1.0, 100.0)
        self.ddynrec.add_variable("rollI", "rollP integral scale", 0.0, -1.0, 100.0)
        self.ddynrec.add_variable("rollD", "rollP derrivative scale", 0.0, -1.0, 100.0)

        self.ddynrec.add_variable("xAcelP", "xAcelP proportion scale", 0.0, -1.0, 1000.0)
        self.ddynrec.add_variable("xAcelI", "xAcelP integral scale", 0.0, -1.0, 1000.0)
        self.ddynrec.add_variable("xAcelD", "xAcelP derrivative scale", 0.0, -1.0, 1000.0)

        self.ddynrec.add_variable("yAcelP", "yAcelP proportion scale", 0.0, -1.0, 100.0)
        self.ddynrec.add_variable("yAcelI", "yAcelP integral scale", 0.0, -1.0, 100.0)
        self.ddynrec.add_variable("yAcelD", "yAcelP derrivative scale", 0.0, -1.0, 100.0)

        self.ddynrec.add_variable("zAcelP", "zAcelP proportion scale", 0.0, -1.0, 100.0)
        self.ddynrec.add_variable("zAcelI", "zAcelP integral scale", 0.0, -1.0, 100.0)
        self.ddynrec.add_variable("zAcelD", "zAcelP derrivative scale", 0.0, -1.0, 100.0)

        self.ddynrec.add_variable("upperLimit", "pid limit", 0.5, -1.0, 5.0)

        self.ddynrec.start(self.dyn_rec_callback)

    def dyn_rec_callback(self, config, data):

        self.yawPID = PID(config["yawP"], config["yawI"], config["yawD"])
        self.pitchPID = PID(config["rollP"], config["rollI"], config["rollD"])
        self.xAcelPID = PID(config["xAcelP"], config["xAcelI"], config["xAcelD"])
        self.yAcelPID = PID(config["yAcelP"], config["yAcelI"], config["yAcelD"])
        self.zAcelPID = PID(config["zAcelP"], config["zAcelI"], config["zAcelD"])

        self.yawPID.setSampleTime(0.1)
        self.pitchPID.setSampleTime(0.1)
        self.xAcelPID.setSampleTime(0.1)
        self.yAcelPID.setSampleTime(0.1)
        self.zAcelPID.setSampleTime(0.1)

        self.yawPID.setOutputLimits(config["upperLimit"])
        self.pitchPID.setOutputLimits(config["upperLimit"])
        self.xAcelPID.setOutputLimits(config["upperLimit"])
        self.yAcelPID.setOutputLimits(config["upperLimit"])
        self.zAcelPID.setOutputLimits(config["upperLimit"])

        return config

if __name__ == '__main__':
    pid() 
