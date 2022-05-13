#!/usr/bin/env python3

from curses import update_lines_cols
from simple_pid import PID
import rospy

from sensor_msgs.msg import Imu, MagneticField
from wurov.msg import trajectory


class simulate_imu_data:
    def __init__(self):  
        rospy.init_node('pid', anonymous=True)

        rospy.Subscriber('imu/data', Imu, self.updateCurrent)
        rospy.Subscriber('trajectory_request', trajectory, self.updateSetpoint)

        self._publisher = rospy.Publisher('trajectory_corrected', trajectory, queue_size=3)

        rospy.Timer(rospy.Duration(0.1), self.pid)

        self.yawPID = PID(1, 0.1, 0.05)
        self.pitchPID = PID(1, 0.1, 0.05)
        self.xAcelPID = PID(1, 0.1, 0.05)
        self.yAcelPID = PID(1, 0.1, 0.05)
        self.zAcelPID = PID(1, 0.1, 0.05)

        rospy.spin()

    def updateCurrent(self, data):
        self.currentPitch = data.orientation.pitch
        self.currentYaw = data.orientation.yaw
        self.currentAccel_x = data.translation.x
        self.currentAccel_y = data.translation.y
        self.currentAccel_z = data.translation.z


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

if __name__ == '__main__':
    simulate_imu_data() 

