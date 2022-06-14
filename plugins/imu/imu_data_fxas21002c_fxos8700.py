#!/usr/bin/env python3

import string
import board;
import busio;
import adafruit_fxas21002c;
import adafruit_fxos8700;
import rospy
from sensor_msgs.msg import Imu, MagneticField



class ImuData:
    def __init__(self, publish=True):
        # init hardwares
        i2c             = busio.I2C(board.SCL, board.SDA)
        self.gyroSensor = adafruit_fxas21002c.FXAS21002C(i2c)
        self.sensor     = adafruit_fxos8700.FXOS8700(i2c)

        # init messages
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()
        self.imu_raw_msg = Imu()
        self.mag_raw_msg = MagneticField()
        
        # init offset values
        self.linear_accel_offset    = rospy.get_param("~linear_accel_offset")
        self.angular_vel_offset     = rospy.get_param("~angular_vel_offset")
        self.magnetic_field_offset  = rospy.get_param("~magnetic_field_offset")

        # zeros matrix for unknow covariance according to sensor_msgs/Imu doc
        zeros_mat = [0]*9
        self.imu_msg.orientation_covariance         = zeros_mat
        self.imu_msg.angular_velocity_covariance    = zeros_mat
        self.imu_msg.linear_acceleration_covariance = zeros_mat

        if publish:
            # imu_filter_madgwick input topics
            rospy.init_node('imu_raw_data', anonymous=True)
            self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=3)
            self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=3)

            rospy.Timer(rospy.Duration(0.1), self.publish_imu)
            rospy.spin()

    def publish_imu(self, event) -> None:
        time = rospy.Time.now()
        self.populate_imu_corrected(time)
        self.populate_mag_corrected(time)
        self.imu_pub.publish(self.imu_msg)
        self.mag_pub.publish(self.mag_msg)

    def populate_imu_corrected(self, time=rospy.Time.now()) -> Imu:
        # Sensor readings
        accel_x, accel_y, accel_z   = self.read_sensor("accelerometer")                   # in m/s^2
        ang_x, ang_y, ang_z         = self.read_sensor("gyroscope")                       # in Radians/s
        # Populate IMU message
        self.imu_msg.header.stamp           = time
        self.imu_msg.header.frame_id        = 'base_link'
        self.imu_msg.linear_acceleration.x  = accel_x - self.linear_accel_offset['x']
        self.imu_msg.linear_acceleration.y  = accel_y - self.linear_accel_offset['y']
        self.imu_msg.linear_acceleration.z  = accel_z - self.linear_accel_offset['z']    # negative absolute is to ensure z-axis is always -9.8 m/s initally
        self.imu_msg.angular_velocity.x     = ang_x - self.angular_vel_offset['x']
        self.imu_msg.angular_velocity.y     = ang_y - self.angular_vel_offset['y']
        self.imu_msg.angular_velocity.z     = ang_z - self.angular_vel_offset['z']
        return self.imu_msg # Optionally return the message

    def populate_mag_corrected(self, time=rospy.Time.now()) -> MagneticField:
        # Sensor readings
        mag_x, mag_y, mag_z = [k/1000000 for k in self.read_sensor("magnetometer")]      # in Tesla
        # Populate Mag message
        self.mag_msg.header.stamp       = time
        self.mag_msg.header.frame_id    = 'base_link'
        self.mag_msg.magnetic_field.x   = mag_x - self.magnetic_field_offset['x']
        self.mag_msg.magnetic_field.y   = mag_y - self.magnetic_field_offset['y']
        self.mag_msg.magnetic_field.z   = mag_z - self.magnetic_field_offset['z']
        return self.mag_msg # Optionally return the message

    def populate_imu_raw(self, time=rospy.Time.now()) -> Imu:
        # Sensor readings
        accel_x, accel_y, accel_z   = self.read_sensor("accelerometer")                         # in m/s^2
        ang_x, ang_y, ang_z         = self.read_sensor("gyroscope")                             # in Radians/s
        # Populate IMU message
        self.imu_raw_msg.header.stamp           = time
        self.imu_raw_msg.header.frame_id        = 'base_link'
        self.imu_raw_msg.linear_acceleration.x  = accel_x
        self.imu_raw_msg.linear_acceleration.y  = accel_y
        self.imu_raw_msg.linear_acceleration.z  = accel_z
        self.imu_raw_msg.angular_velocity.x     = ang_x
        self.imu_raw_msg.angular_velocity.y     = ang_y
        self.imu_raw_msg.angular_velocity.z     = ang_z
        return self.imu_raw_msg # Optionally return the message
    
    def populate_mag_raw(self, time=rospy.Time.now()) -> MagneticField:
        # Sensor readings
        mag_x, mag_y, mag_z= [k/1000000 for k in self.read_sensor("magnetometer")]      # in Tesla
        # Populate IMU message
        self.mag_raw_msg.header.stamp       = time
        self.mag_raw_msg.header.frame_id    = 'base_link'
        self.mag_raw_msg.magnetic_field.x   = mag_x
        self.mag_raw_msg.magnetic_field.y   = mag_y
        self.mag_raw_msg.magnetic_field.z   = mag_z
        return self.mag_raw_msg # Optionally return the message

    def read_sensor(self, sensor: string):
        """Read sensor and correct its orientation"""
        # REP103:
        # +x: forward
        # +y: left
        # +z: up

        # From the board (06/13/2022)
        # REP103 = Board
        # +x = -z
        # +y = x
        # +z = -y
        if sensor == "accelerometer":
            sensor_z, sensor_x, sensor_y = self.sensor.accelerometer
            return -sensor_x, sensor_y, -sensor_z
        #TODO: Fix this orientation
        elif sensor == "magnetometer":
            sensor_y, sensor_z, sensor_x = self.sensor.magnetometer
            return sensor_x, sensor_y, sensor_z
        #TODO: Fix this orientation
        elif sensor == "gyroscope":
            sensor_y, sensor_z, sensor_x = self.gyroSensor.gyroscope
            return sensor_x, sensor_y, sensor_z
        return None

    def set_imu_offsets(self, accel: dict, ang: dict, mag: dict) -> None:
        self.linear_accel_offset    = accel
        self.angular_vel_offset     = ang
        self.magnetic_field_offset  = mag        
        
if __name__ == '__main__':
    ImuData()