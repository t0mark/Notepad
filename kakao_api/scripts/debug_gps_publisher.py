#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Debug GPS Publisher

디버깅용 가상 GPS 데이터 발행 노드
"""

import rospy
from sensor_msgs.msg import NavSatFix

def publish_gps():
    rospy.init_node('debug_gps_publisher', anonymous=True)

    gps_pub = rospy.Publisher('/ublox/fix', NavSatFix, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    # 초기 위치: 35.846171, 127.134468
    lat = 35.846171
    lon = 127.134468
    alt = 100.0

    rospy.loginfo("🛰️  Debug GPS Publisher 시작")
    rospy.loginfo(f"   위치: ({lat}, {lon})")
    rospy.loginfo(f"   고도: {alt}m")
    rospy.loginfo("   1Hz로 GPS 데이터 발행 중...")

    while not rospy.is_shutdown():
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "gps_link"

        msg.status.status = 0  # GPS fix
        msg.status.service = 1

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        msg.position_covariance = [1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0]
        msg.position_covariance_type = 1

        gps_pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_gps()
    except rospy.ROSInterruptException:
        rospy.loginfo("Debug GPS Publisher 종료")
