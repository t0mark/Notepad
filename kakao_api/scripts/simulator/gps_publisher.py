#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GPS Publisher for Simulation/Testing

테스트/시뮬레이션용 GPS 발행 노드
- 고정된 GPS 좌표를 /ublox/fix로 발행
- 외부 GPS 발행자 감지 시 자동 중지
"""

import rospy
from sensor_msgs.msg import NavSatFix


class GPSPublisher:
    def __init__(self):
        # Publisher
        self.gps_pub = rospy.Publisher('/ublox/fix', NavSatFix, queue_size=10)

        self.is_publishing = True
        self.my_seq = 0

        # 다른 발행자 감지를 위한 구독자
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback, queue_size=10)

        # 초기 위치
        self.lat = 35.846171
        self.lon = 127.134468
        self.alt = 100.0

    def gps_callback(self, msg):
        """자신이 발행한 메시지가 아닌 경우 감지"""
        if msg.header.frame_id != "gps_link_sim":
            if self.is_publishing:
                rospy.logwarn("=" * 60)
                rospy.logwarn("⚠️  외부 GPS 발행자 감지!")
                rospy.logwarn(f"   Frame ID: {msg.header.frame_id}")
                rospy.logwarn("   GPS Publisher를 중지합니다.")
                rospy.logwarn("=" * 60)
                self.is_publishing = False

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz

        rospy.loginfo("=" * 60)
        rospy.loginfo("🛰️  GPS Publisher 시작")
        rospy.loginfo(f"   토픽: /ublox/fix")
        rospy.loginfo(f"   위치: ({self.lat}, {self.lon})")
        rospy.loginfo(f"   고도: {self.alt}m")
        rospy.loginfo(f"   주파수: 1Hz")
        rospy.loginfo("")
        rospy.loginfo("   (외부 GPS 발행자 감지 시 자동 중지)")
        rospy.loginfo("=" * 60)

        # 초기 대기 (외부 GPS가 먼저 시작될 수 있도록)
        rospy.sleep(2.0)

        while not rospy.is_shutdown():
            if self.is_publishing:
                # GPS 메시지 발행
                gps_msg = NavSatFix()
                gps_msg.header.stamp = rospy.Time.now()
                gps_msg.header.frame_id = "gps_link_sim"
                gps_msg.header.seq = self.my_seq
                self.my_seq += 1

                gps_msg.status.status = 0  # GPS fix
                gps_msg.status.service = 1

                gps_msg.latitude = self.lat
                gps_msg.longitude = self.lon
                gps_msg.altitude = self.alt

                gps_msg.position_covariance = [1.0, 0.0, 0.0,
                                               0.0, 1.0, 0.0,
                                               0.0, 0.0, 1.0]
                gps_msg.position_covariance_type = 1

                self.gps_pub.publish(gps_msg)

                # 10초마다 로그 출력
                if int(rospy.Time.now().to_sec()) % 10 == 0:
                    rospy.loginfo(f"📡 GPS 발행 중: ({self.lat:.6f}, {self.lon:.6f})")

            rate.sleep()


def main():
    rospy.init_node('gps_publisher', anonymous=True)
    publisher = GPSPublisher()
    publisher.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPS Publisher 종료")
