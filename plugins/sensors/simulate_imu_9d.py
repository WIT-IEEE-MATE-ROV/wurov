import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField


class simulate_imu_9d:
    def __init__(self):
        rospy.init_node('simulate_imu_9d', anonymous=True)

        # imu_filter_madgwick's required topics
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=3)
        self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=3)

        rospy.Timer(rospy.Duration(0.1), self.publisher)

        rospy.spin()

    def publisher(self, data):
        imu_msg = Imu()
        mag_msg = MagneticField()

        # While this node is still running, keep getting sensor values
        # In this case, it's simulated, so we're making up values
        # In your case, replace this block with however you're getting values
        # A real sensor is likley providing accelerometer, gyroscope, and magnetometer values
        # Combine them to produce more accure roll/pitch/yaw/x/y/z values
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'odom'        

        imu_msg.orientation.x = np.random.normal()
        imu_msg.orientation.y = np.random.normal()
        imu_msg.orientation.z = np.random.normal()
        imu_msg.orientation.w = np.random.normal()

        imu_msg.linear_acceleration.x = np.random.normal()
        imu_msg.linear_acceleration.y = np.random.normal()
        imu_msg.linear_acceleration.z = np.random.normal() 

        imu_msg.angular_velocity.x = np.random.normal()
        imu_msg.angular_velocity.y = np.random.normal()
        imu_msg.angular_velocity.z = np.random.normal()


        # no initial covariance
        # imu_msg.orientation_covariance[0] = -1
        # imu_msg.angular_velocity_covariance[0] = -1
        # imu_msg.linear_acceleration_covariance[0] = -1

        iden = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        imu_msg.orientation_covariance = iden
        imu_msg.angular_velocity_covariance = iden
        imu_msg.linear_acceleration_covariance = iden

        mag_msg.magnetic_field.x = np.random.normal()
        mag_msg.magnetic_field.y = np.random.normal()
        mag_msg.magnetic_field.z = np.random.normal()

        # publish msgs
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)

if __name__ == '__main__':
    simulate_imu_9d()