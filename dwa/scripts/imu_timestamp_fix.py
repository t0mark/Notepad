#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    # Change timestamp to real time
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('imu_timestamp_fix')

    # Subscribe to Gazebo IMU (with sim time)
    rospy.Subscriber('/gazebo_time_imu', Imu, imu_callback)

    # Publish to /ouster/imu with real time
    pub = rospy.Publisher('/ouster/imu', Imu, queue_size=10)

    rospy.spin()
