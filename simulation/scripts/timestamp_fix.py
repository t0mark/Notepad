#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2

def imu_callback(msg):
    # Change timestamp to real time
    msg.header.stamp = rospy.Time.now()
    imu_pub.publish(msg)

def gps_callback(msg):
    # Change timestamp to real time
    msg.header.stamp = rospy.Time.now()
    gps_pub.publish(msg)

def pointcloud_callback(msg):
    # Change timestamp to real time
    msg.header.stamp = rospy.Time.now()
    pointcloud_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('timestamp_fix')

    # Subscribe to Gazebo topics
    rospy.Subscriber('/gazebo_time/imu', Imu, imu_callback)
    rospy.Subscriber('/gazebo_time/gps', NavSatFix, gps_callback)
    rospy.Subscriber('/gazebo_time/points', PointCloud2, pointcloud_callback)
    
    # Publishers with real time
    imu_pub = rospy.Publisher('/ouster/imu', Imu, queue_size=10)
    gps_pub = rospy.Publisher('/ublox/fix', NavSatFix, queue_size=10)
    pointcloud_pub = rospy.Publisher('/ouster/points', PointCloud2, queue_size=10)

    rospy.spin()
