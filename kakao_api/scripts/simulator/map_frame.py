#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Map Frame Definition & Waypoint Converter

Map 프레임 정의 및 웨이포인트 변환 노드 (INS 없이 사용 시)
- /kakao/waypoints (GeoPath) → /waypoints (MarkerArray) 변환
- 첫 GPS 위치를 map 원점으로 설정
- RViz 시각화용 MarkerArray 발행
"""

import rospy
import utm
from geographic_msgs.msg import GeoPath
from visualization_msgs.msg import Marker, MarkerArray


class MapFrameConverter:
    def __init__(self):
        # Publisher
        self.waypoints_pub = rospy.Publisher('/waypoints', MarkerArray, queue_size=10, latch=True)

        # Subscriber
        rospy.Subscriber('/kakao/waypoints', GeoPath, self.waypoints_callback)

        # Map 원점 설정
        self.map_origin_utm = None
        self.utm_zone_number = None
        self.utm_zone_letter = None
        self.origin_set = False

        rospy.loginfo("=" * 60)
        rospy.loginfo("🗺️  Map Frame Converter 시작")
        rospy.loginfo("   /kakao/waypoints → /waypoints 변환")
        rospy.loginfo("   첫 웨이포인트로 map 원점 자동 설정")
        rospy.loginfo("=" * 60)

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
        rospy.loginfo(f"✅ Map 프레임 변환 완료: {len(msg.poses)}개 → /waypoints")


def main():
    rospy.init_node('map_frame_converter', anonymous=True)
    converter = MapFrameConverter()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Map Frame Converter 종료")
