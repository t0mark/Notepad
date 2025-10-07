#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Debug GPS and Map Publisher

INS 없이 독립적으로 사용할 때 필요한 디버깅 노드
- GPS 데이터를 /ublox/fix로 발행
- /kakao/waypoints (GeoPath) → /waypoints (MarkerArray) 변환 (디버깅용 map 프레임)
- 첫 GPS 위치를 map 원점으로 설정
- 다른 노드가 GPS를 발행하면 자동으로 멈춤
"""

import rospy
import utm
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPath
from visualization_msgs.msg import Marker, MarkerArray


class DebugGPSMapPublisher:
    def __init__(self):
        # Publishers
        self.gps_pub = rospy.Publisher('/ublox/fix', NavSatFix, queue_size=10)
        self.waypoints_pub = rospy.Publisher('/waypoints', MarkerArray, queue_size=10, latch=True)

        self.is_publishing = True
        self.my_seq = 0

        # 다른 발행자 감지를 위한 구독자
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback, queue_size=10)

        # GeoPath를 MarkerArray로 변환
        rospy.Subscriber('/kakao/waypoints', GeoPath, self.waypoints_callback)

        # 초기 위치: 35.846171, 127.134468 (INS datum과 동일)
        self.lat = 35.84617083648474
        self.lon = 127.13446738445664
        self.alt = 100.0

        # Map 원점 설정
        self.map_origin_utm = None
        self.utm_zone_number = None
        self.utm_zone_letter = None
        self.origin_set = False

    def set_map_origin(self, lat, lon):
        """첫 GPS 위치를 map 원점으로 설정"""
        if self.origin_set:
            return

        try:
            easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)

            self.map_origin_utm = {
                'easting': easting,
                'northing': northing,
                'lat': lat,
                'lon': lon
            }
            self.utm_zone_number = zone_num
            self.utm_zone_letter = zone_letter
            self.origin_set = True

            rospy.loginfo("=" * 60)
            rospy.loginfo("🎯 Map 프레임 원점 설정 완료!")
            rospy.loginfo(f"   GPS: ({lat:.6f}, {lon:.6f})")
            rospy.loginfo(f"   UTM: ({easting:.2f}, {northing:.2f})")
            rospy.loginfo(f"   Zone: {zone_num}{zone_letter}")
            rospy.loginfo(f"   Map 원점 = (0, 0)")
            rospy.loginfo("=" * 60)

        except Exception as e:
            rospy.logerr(f"❌ Map 원점 설정 실패: {e}")

    def gps_to_map(self, lat, lon):
        """GPS 좌표를 map 프레임 좌표로 변환"""
        if not self.origin_set:
            # 첫 GPS를 원점으로 설정
            self.set_map_origin(lat, lon)
            return 0.0, 0.0

        try:
            easting, northing, _, _ = utm.from_latlon(lat, lon)
            map_x = easting - self.map_origin_utm['easting']
            map_y = northing - self.map_origin_utm['northing']
            return map_x, map_y
        except Exception as e:
            rospy.logerr(f"❌ GPS → map 변환 실패: {e}")
            return None, None

    def waypoints_callback(self, msg):
        """/kakao/waypoints (GeoPath) → /waypoints (MarkerArray) 변환"""
        if len(msg.poses) == 0:
            rospy.logwarn("⚠️  빈 웨이포인트 수신")
            return

        rospy.loginfo(f"📥 GPS 웨이포인트 수신: {len(msg.poses)}개")

        # 첫 번째 웨이포인트로 map 원점 설정 (아직 설정 안 됐을 때만)
        first_wp = msg.poses[0]
        if not self.origin_set:
            self.set_map_origin(first_wp.pose.position.latitude,
                              first_wp.pose.position.longitude)

        # MarkerArray 생성
        marker_array = MarkerArray()

        # 1. 웨이포인트 큐브 마커
        for i, geo_pose in enumerate(msg.poses):
            lat = geo_pose.pose.position.latitude
            lon = geo_pose.pose.position.longitude
            alt = geo_pose.pose.position.altitude

            map_x, map_y = self.gps_to_map(lat, lon)

            if map_x is not None and map_y is not None:
                # Marker 생성
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "map"
                marker.ns = "waypoints"
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                marker.pose.position.x = map_x
                marker.pose.position.y = map_y
                marker.pose.position.z = 0.0
                marker.pose.orientation = geo_pose.pose.orientation

                # 크기
                marker.scale.x = 2.0
                marker.scale.y = 2.0
                marker.scale.z = 3.0

                # 색상 (마젠타)
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.8

                marker.lifetime = rospy.Duration(0)  # 영구

                marker_array.markers.append(marker)

                # 로깅 (처음 3개와 마지막 3개만)
                if i < 3 or i >= len(msg.poses) - 3:
                    rospy.loginfo(f"   WP{i+1}: GPS({lat:.6f}, {lon:.6f}) → Map({map_x:.1f}, {map_y:.1f})")
                elif i == 3 and len(msg.poses) > 6:
                    rospy.loginfo(f"   ... (중간 {len(msg.poses)-6}개 생략)")

        # 2. 연결선 마커 (LINE_STRIP)
        if len(marker_array.markers) > 1:
            line_marker = Marker()
            line_marker.header.stamp = rospy.Time.now()
            line_marker.header.frame_id = "map"
            line_marker.ns = "waypoint_path"
            line_marker.id = 1000
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            # 모든 웨이포인트를 선으로 연결
            for marker in marker_array.markers:
                line_marker.points.append(marker.pose.position)

            # 선 두께
            line_marker.scale.x = 0.5

            # 색상 (노란색)
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0

            line_marker.lifetime = rospy.Duration(0)

            marker_array.markers.append(line_marker)

        # MarkerArray 발행
        self.waypoints_pub.publish(marker_array)
        rospy.loginfo(f"✅ Map 프레임 변환 완료: {len(msg.poses)}개 → /waypoints (MarkerArray)")

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
        rospy.loginfo(f"   위치: ({self.lat}, {self.lon})")
        rospy.loginfo(f"   고도: {self.alt}m")
        rospy.loginfo("")
        rospy.loginfo("   기능:")
        rospy.loginfo("   1. 고정 GPS 발행 (1Hz)")
        rospy.loginfo("   2. /kakao/waypoints → /waypoints 변환 (MarkerArray)")
        rospy.loginfo("   3. 첫 GPS를 map 원점으로 설정")
        rospy.loginfo("")
        rospy.loginfo("   (외부 GPS 발행자 감지 시 자동 중지)")
        rospy.loginfo("=" * 60)

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

                # 10초마다 로그 출력
                if int(rospy.Time.now().to_sec()) % 10 == 0:
                    if self.origin_set:
                        rospy.loginfo(f"📡 GPS 발행 중 | Map 원점: ({self.map_origin_utm['lat']:.6f}, {self.map_origin_utm['lon']:.6f})")
                    else:
                        rospy.loginfo(f"📡 GPS 발행 중 | Map 원점: 미설정 (웨이포인트 대기 중)")

            rate.sleep()


def main():
    rospy.init_node('gps_debug', anonymous=True)
    publisher = DebugGPSMapPublisher()
    publisher.publish_gps()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Debug GPS & Map Publisher 종료")
