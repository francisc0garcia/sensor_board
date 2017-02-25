#!/usr/bin/env python

# Import ROS
import rospy
from geometry_msgs.msg import Quaternion, Twist
import time
from math import sin, cos, pi
import numpy as np
from tf.broadcaster import TransformBroadcaster

import pigpio

#https://github.com/ghirlekar/bno055-python-i2c

# import libraries
from classes.BNO055 import *
from classes.imu_data import *
from classes.imu_publisher import *

class RobotImuPublisherNode:
    def __init__(self):
        rospy.loginfo('Starting RobotImuPublisherNode')

        #self.br = tf.TransformBroadcaster()

        self.odomBroadcaster_imu_1 = TransformBroadcaster()
        self.odomBroadcaster_imu_2 = TransformBroadcaster()
        self.odomBroadcaster_gps = TransformBroadcaster()

        # create object to store robot data
        self.sensor_imu_data = ImuData()

        # Get values from launch file
        self.offset_pitch = rospy.get_param('~offset_pitch', 0)
        self.offset_roll = rospy.get_param('~offset_roll', 0)
        self.offset_yaw = rospy.get_param('~offset_yaw', 0)
        self.rate = rospy.get_param('~rate', 100.0)  # the rate at which to publish the transform
        self.gravity_offset = -1

        self.calibration_file = 'calibration.json'
        rospy.loginfo("IMU offset pitch: %d roll: %d yaw: %d",  self.offset_pitch, self.offset_roll, self.offset_yaw)


        # Initialize IMU's
        self.imu_1 = ImuData(address=BNO055_ADDRESS_A)
        self.imu_1.init_imu()

        self.imu_2 = ImuData(address=BNO055_ADDRESS_B)
        self.imu_2.init_imu()

        # Create a publisher for imu message
        self.imu_pub_1 = ImuPublisher(topic_name = '/sensor/imu_1', frame_name='frame_imu_1')
        self.imu_pub_2 = ImuPublisher(topic_name = '/sensor/imu_2', frame_name='frame_imu_2')

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        self.seq = 0

        rospy.on_shutdown(self.shutdown_node)
        rate = rospy.Rate(self.rate)

        rospy.loginfo("Ready for publishing")

        # Main while loop.
        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()

            # Read and publish imu 1
            self.imu_pub_1.publish_imu(self.imu_1, self.seq)

            # Read and publish imu 2
            self.imu_pub_2.publish_imu(self.imu_2, self.seq)

            # publish tf between IMU frames
            self.th = 0
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )

            self.odomBroadcaster_imu_1.sendTransform(
                (0.0, 0.0, 0.5),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(), self.imu_pub_1.frame_name, "world"
            )

            self.odomBroadcaster_imu_2.sendTransform(
                (1.0, 0.0, 0.5),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(), self.imu_pub_2.frame_name, "world"
            )

            self.odomBroadcaster_gps.sendTransform(
                (0.0, 0.0, 0.0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(), 'gps', "world"
            )

            self.seq += 1

            rate.sleep()

    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_imu_publisher")

# Main function.
if __name__ == '__main__':
    #from Adafruit_BNO055 import BNO055
    time.sleep(2)
    pi = pigpio.pi()

    while not pi.connected:
        rospy.loginfo('Failed to initialize pigpio. Trying again...')
        time.sleep(1)
        pi = pigpio.pi()

    # Initialize the node and name it.
    rospy.init_node('sensor_imu_publisher')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        obj_temp = RobotImuPublisherNode()
    except rospy.ROSInterruptException:
        pass
