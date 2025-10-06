#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Debug GPS Publisher

디버깅용 가상 GPS 데이터 발행 노드
다른 노드가 GPS를 발행하기 시작하면 자동으로 멈춤
"""

import rospy
from sensor_msgs.msg import NavSatFix

class DebugGPSPublisher:
    def __init__(self):
        self.gps_pub = rospy.Publisher('/ublox/fix', NavSatFix, queue_size=10)
        self.is_publishing = True
        self.my_seq = 0

        # 다른 발행자 감지를 위한 구독자
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback, queue_size=10)

        # 초기 위치: 35.846171, 127.134468
        self.lat = 35.846171
        self.lon = 127.134468
        self.alt = 100.0

    def gps_callback(self, msg):
        # 자신이 발행한 메시지가 아닌 경우 (frame_id로 구분)
        if msg.header.frame_id != "gps_link_debug":
            if self.is_publishing:
                rospy.logwarn("=" * 60)
                rospy.logwarn("⚠️  외부 GPS 발행자 감지!")
                rospy.logwarn(f"   Frame ID: {msg.header.frame_id}")
                rospy.logwarn("   Debug GPS Publisher를 중지합니다.")
                rospy.logwarn("=" * 60)
                self.is_publishing = False

    def publish_gps(self):
        rate = rospy.Rate(1)  # 1 Hz

        rospy.loginfo("=" * 60)
        rospy.loginfo("🛰️  Debug GPS Publisher 시작")
        rospy.loginfo(f"   토픽: /ublox/fix")
        rospy.loginfo(f"   위치: ({self.lat}, {self.lon})")
        rospy.loginfo(f"   고도: {self.alt}m")
        rospy.loginfo("   1Hz로 GPS 데이터 발행 중...")
        rospy.loginfo("   (외부 GPS 발행자 감지 시 자동 중지)")
        rospy.loginfo("=" * 60)

        # 초기 대기 (Gazebo GPS 플러그인이 먼저 시작될 수 있도록)
        rospy.sleep(2.0)

        while not rospy.is_shutdown():
            if self.is_publishing:
                msg = NavSatFix()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "gps_link_debug"  # 구분을 위해 다른 frame_id 사용
                msg.header.seq = self.my_seq
                self.my_seq += 1

                msg.status.status = 0  # GPS fix
                msg.status.service = 1

                msg.latitude = self.lat
                msg.longitude = self.lon
                msg.altitude = self.alt

                msg.position_covariance = [1.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0,
                                           0.0, 0.0, 1.0]
                msg.position_covariance_type = 1

                self.gps_pub.publish(msg)

                # 5초마다 로그 출력
                if int(rospy.Time.now().to_sec()) % 5 == 0:
                    rospy.loginfo(f"📡 GPS 발행: ({self.lat}, {self.lon})")

            rate.sleep()

def publish_gps():
    rospy.init_node('debug_gps_publisher', anonymous=True)
    publisher = DebugGPSPublisher()
    publisher.publish_gps()

if __name__ == '__main__':
    try:
        publish_gps()
    except rospy.ROSInterruptException:
        rospy.loginfo("Debug GPS Publisher 종료")
