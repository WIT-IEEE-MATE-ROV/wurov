#!/usr/bin/env python3

import argparse
from asyncio import current_task
from calendar import c
from time import time
import rospy
from sensor_msgs.msg import Imu, MagneticField
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from imu_data_fxas21002c_fxos8700 import ImuData
from wurov.msg import imu_offset

class ImuCalibration:
    def __init__(self) -> None:
        # NOTE: Only recalibrate IMU when the IMU is oriented correctly and when the vehicle is on a flat surface
        # arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser("IMU Calibration")
        parser.add_argument('--accel_calibration', type=bool, default=True, help='set to true if calibrating Accelerometer')
        parser.add_argument('--gyro_calibration', type=bool, default=True, help='set to true if calibrating Gyroscope')
        parser.add_argument('--mag_calibration', type=bool, default=False, help='set to true if calibrating Magnetometer')
        parser.add_argument('--dynamic_calibration', type=bool, default=False, help='set to true if dynamically calibrating the IMU')
        self.args = parser.parse_args(rospy.myargv()[1:])


        # imu_filter_madgwick input topics
        rospy.init_node('imu_calibration', anonymous=True, log_level=rospy.DEBUG)
        self.imu_raw_pub    = rospy.Publisher('imu/data_no_offsets', Imu, queue_size=3)
        self.mag_raw_pub    = rospy.Publisher('imu/mag_no_offsets', MagneticField, queue_size=3)
        self.imu_pub = rospy.Publisher('imu/data_with_offsets', Imu, queue_size=3)
        self.mag_pub = rospy.Publisher('imu/mag_with_offsets', MagneticField, queue_size=3)
        self.offset_pub = rospy.Publisher('imu/offsets', imu_offset, queue_size=3)
        
        # ImuData to access sensor data
        self.imu = ImuData(publish=False)

        # Getting data
        self.data = self.sensor_data_over_period()

        # init offset values
        #TODO: Publish this to debug
        self.linear_accel_offset    = rospy.get_param("~linear_accel_offset")
        self.angular_vel_offset     = rospy.get_param("~angular_vel_offset")
        self.magnetic_field_offset  = rospy.get_param("~magnetic_field_offset")
        self.imu_offset = imu_offset()

        rospy.loginfo(self.args.accel_calibration)
        rospy.loginfo(self.args.gyro_calibration)
        rospy.loginfo(self.args.mag_calibration)

        if self.args.dynamic_calibration:
            pass
        else:
            if type(self.args.accel_calibration) == type(True):
                self.calculate_accel_offset(self.data['accel'])
            if self.args.gyro_calibration == True:
                self.calculate_angular_offset(self.data['ang'])
            if self.args.mag_calibration == True:
                self.calculate_mag_offset(self.data['mag'])
            self.write_offsets_to_param()
            self.imu.set_imu_offsets(self.linear_accel_offset, self.angular_vel_offset, self.magnetic_field_offset)

        rospy.Timer(rospy.Duration(0.1), self.publish_all)
        rospy.spin()

    def publish_all(self, event):
        current_time = rospy.Time.now()
        imu_msg            = self.imu.populate_imu_corrected(current_time)
        imu_no_offset_msg  = self.imu.populate_imu_raw(current_time)
        mag_msg            = self.imu.populate_mag_corrected(current_time)
        mag_no_offset_msg  = self.imu.populate_mag_raw(current_time)
        imu_offset         = self.populate_imu_offset(current_time)

        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)
        self.imu_raw_pub.publish(imu_no_offset_msg)
        self.mag_raw_pub.publish(mag_no_offset_msg)
        self.offset_pub.publish(imu_offset)
    
    def populate_imu_offset(self, time):
        self.imu_offset.header.stamp = time
        self.imu_offset.header.frame_id    = 'base_link'
        self.imu_offset.accel_offset.x  = self.linear_accel_offset['x']
        self.imu_offset.accel_offset.y  = self.linear_accel_offset['y']
        self.imu_offset.accel_offset.z  = self.linear_accel_offset['z']
        self.imu_offset.ang_offset.x    = self.angular_vel_offset['x']
        self.imu_offset.ang_offset.y    = self.angular_vel_offset['y']
        self.imu_offset.ang_offset.z    = self.angular_vel_offset['z']
        self.imu_offset.mag_offset.x    = self.magnetic_field_offset['x']
        self.imu_offset.mag_offset.y    = self.magnetic_field_offset['y']
        self.imu_offset.mag_offset.z    = self.magnetic_field_offset['z']
        return self.imu_offset

    def sensor_data_over_period(self, samples_num=20) -> dict:
        accel   = {'x': [], 'y': [], 'z': []}
        ang     = {'x': [], 'y': [], 'z': []}
        mag     = {'x': [], 'y': [], 'z': []}
        for _ in range(samples_num):
            current_time = rospy.Time.now()
            raw_imu = self.imu.populate_imu_raw(current_time)
            raw_mag = self.imu.populate_mag_raw(current_time)
            accel['x'].append(raw_imu.linear_acceleration.x)
            accel['y'].append(raw_imu.linear_acceleration.y)
            accel['z'].append(raw_imu.linear_acceleration.z)
            ang['x'].append(raw_imu.angular_velocity.x)
            ang['y'].append(raw_imu.angular_velocity.y)
            ang['z'].append(raw_imu.angular_velocity.z)
            mag['x'].append(raw_mag.magnetic_field.x)
            mag['y'].append(raw_mag.magnetic_field.y)
            mag['z'].append(raw_mag.magnetic_field.z)
        return {'accel': accel, 'ang': ang, 'mag': mag}
        
    def calculate_accel_offset(self, data):
        self.linear_accel_offset['x'] = sum(data['x'])/len(data['x'])
        self.linear_accel_offset['y'] = sum(data['y'])/len(data['y'])
        self.linear_accel_offset['z'] = sum(data['z'])/len(data['z']) + 9.8

    def calculate_angular_offset(self, data):
        self.angular_vel_offset['x'] = sum(data['x'])/len(data['x'])
        self.angular_vel_offset['y'] = sum(data['y'])/len(data['y'])
        self.angular_vel_offset['z'] = sum(data['z'])/len(data['z'])
    
    #TODO: figure out what is the normal mag data
    def calculate_mag_offset(self,data):
        self.magnetic_field_offset['x'] = sum(data['x'])/len(data['x'])
        self.magnetic_field_offset['y'] = sum(data['y'])/len(data['y'])
        self.magnetic_field_offset['z'] = sum(data['z'])/len(data['z'])
    
    #TODO: maybe find a way to permanently write to the param server
    def write_offsets_to_param(self):
        rospy.set_param('linear_accel_offset', self.linear_accel_offset)
        rospy.set_param('angular_vel_offset', self.angular_vel_offset)
        rospy.set_param('magnetic_field_offset', self.magnetic_field_offset)

if __name__=="__main__":
    imu_calibration = ImuCalibration()
