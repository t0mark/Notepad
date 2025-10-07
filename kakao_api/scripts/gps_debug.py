#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Debug GPS and Map Publisher

디버깅용 가상 GPS 데이터 발행 및 map 프레임 정의 노드
- GPS 데이터를 /ublox/fix로 발행
- 첫 GPS 위치를 map 원점으로 설정하여 /gps_map_pose 발행
- 다른 노드가 GPS를 발행하면 자동으로 멈춤
"""

import rospy
import utm
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class DebugGPSMapPublisher:
    def __init__(self):
        # Publishers
        self.gps_pub = rospy.Publisher('/ublox/fix', NavSatFix, queue_size=10)
        self.map_pose_pub = rospy.Publisher('/gps_map_pose', PoseStamped, queue_size=10)

        self.is_publishing = True
        self.my_seq = 0

        # 다른 발행자 감지를 위한 구독자
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback, queue_size=10)

        # 초기 위치: 35.846171, 127.134468 (INS datum과 동일)
        self.lat = 35.84617083648474
        self.lon = 127.13446738445664
        self.alt = 100.0

        # Map 원점 설정
        self.map_origin_utm = None
        self.utm_zone_number = None
        self.utm_zone_letter = None
        self.origin_set = False

    def set_map_origin(self):
        """첫 GPS 위치를 map 원점으로 설정"""
        try:
            easting, northing, zone_num, zone_letter = utm.from_latlon(self.lat, self.lon)

            self.map_origin_utm = {
                'easting': easting,
                'northing': northing
            }
            self.utm_zone_number = zone_num
            self.utm_zone_letter = zone_letter
            self.origin_set = True

            rospy.loginfo("🎯 Map 프레임 원점 설정 완료!")
            rospy.loginfo(f"   GPS: ({self.lat:.6f}, {self.lon:.6f})")
            rospy.loginfo(f"   UTM: ({easting:.2f}, {northing:.2f})")
            rospy.loginfo(f"   Zone: {zone_num}{zone_letter}")
            rospy.loginfo(f"   Map 원점 = (0, 0)")

        except Exception as e:
            rospy.logerr(f"❌ Map 원점 설정 실패: {e}")

    def gps_to_map(self, lat, lon):
        """GPS 좌표를 map 프레임 좌표로 변환"""
        if not self.origin_set:
            return None, None

        try:
            easting, northing, _, _ = utm.from_latlon(lat, lon)
            map_x = easting - self.map_origin_utm['easting']
            map_y = northing - self.map_origin_utm['northing']
            return map_x, map_y
        except Exception as e:
            rospy.logerr(f"❌ GPS → map 변환 실패: {e}")
            return None, None

    def gps_callback(self, msg):
        """자신이 발행한 메시지가 아닌 경우 감지"""
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
        rospy.loginfo("🛰️  Debug GPS & Map Publisher 시작")
        rospy.loginfo(f"   GPS 토픽: /ublox/fix")
        rospy.loginfo(f"   Map 토픽: /gps_map_pose")
        rospy.loginfo(f"   위치: ({self.lat}, {self.lon})")
        rospy.loginfo(f"   고도: {self.alt}m")
        rospy.loginfo("   1Hz로 GPS 및 Map 데이터 발행 중...")
        rospy.loginfo("   (외부 GPS 발행자 감지 시 자동 중지)")
        rospy.loginfo("=" * 60)

        # Map 원점 설정
        self.set_map_origin()

        # 초기 대기 (Gazebo GPS 플러그인이 먼저 시작될 수 있도록)
        rospy.sleep(2.0)

        while not rospy.is_shutdown():
            if self.is_publishing:
                # GPS 메시지 발행
                gps_msg = NavSatFix()
                gps_msg.header.stamp = rospy.Time.now()
                gps_msg.header.frame_id = "gps_link_debug"
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

                # Map 프레임 좌표 발행
                if self.origin_set:
                    map_x, map_y = self.gps_to_map(self.lat, self.lon)

                    if map_x is not None and map_y is not None:
                        map_msg = PoseStamped()
                        map_msg.header.stamp = rospy.Time.now()
                        map_msg.header.frame_id = "map"

                        map_msg.pose.position.x = map_x
                        map_msg.pose.position.y = map_y
                        map_msg.pose.position.z = 0.0

                        map_msg.pose.orientation.x = 0.0
                        map_msg.pose.orientation.y = 0.0
                        map_msg.pose.orientation.z = 0.0
                        map_msg.pose.orientation.w = 1.0

                        self.map_pose_pub.publish(map_msg)

                # 5초마다 로그 출력
                if int(rospy.Time.now().to_sec()) % 5 == 0:
                    if self.origin_set:
                        rospy.loginfo(f"📡 GPS: ({self.lat:.6f}, {self.lon:.6f}) → Map: ({map_x:.1f}, {map_y:.1f})")
                    else:
                        rospy.loginfo(f"📡 GPS: ({self.lat:.6f}, {self.lon:.6f})")

            rate.sleep()


def publish_gps():
    rospy.init_node('debug_gps_map_publisher', anonymous=True)
    publisher = DebugGPSMapPublisher()
    publisher.publish_gps()


if __name__ == '__main__':
    try:
        publish_gps()
    except rospy.ROSInterruptException:
        rospy.loginfo("Debug GPS & Map Publisher 종료")
