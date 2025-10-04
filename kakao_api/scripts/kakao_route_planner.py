#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Kakao Route Planner Node

카카오 API에서 받은 GPS 웨이포인트를 map 프레임으로 변환하여 발행하는 노드

입력:
  - /waypoints_gps (String): 웹에서 받은 GPS 웨이포인트 (JSON)

출력:
  - /global_waypoints (PoseArray): map 프레임 기준 웨이포인트
  - /kakao_route/status (String): 경로 변환 상태 정보
"""

import rospy
import json
import utm
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


class KakaoRoutePlanner:
    def __init__(self):
        rospy.init_node('kakao_route_planner', anonymous=True)

        # map 프레임 원점 (첫 GPS 수신 시 설정)
        self.map_origin_utm = None
        self.utm_zone_number = None
        self.utm_zone_letter = None
        self.origin_set = False

        # GPS 상태
        self.current_gps = None
        self.gps_available = False

        # Publishers
        self.waypoints_pub = rospy.Publisher('/global_waypoints', PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher('/kakao_route/status', String, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/waypoints_gps', String, self.waypoints_gps_callback)
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)

        # Timer for status publishing
        rospy.Timer(rospy.Duration(5.0), self.publish_status)

        rospy.loginfo("🗺️  Kakao Route Planner 시작")
        rospy.loginfo("📡 GPS 수신 대기 중 (첫 GPS가 map 프레임 원점이 됩니다)")

    def gps_callback(self, msg):
        """GPS 데이터 수신 및 map 프레임 원점 설정"""
        if msg.status.status < 0:
            return

        self.current_gps = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }
        self.gps_available = True

        # 첫 GPS 수신 시 map 프레임 원점 설정
        if not self.origin_set:
            self.set_map_origin(msg.latitude, msg.longitude)

    def set_map_origin(self, lat, lon):
        """map 프레임 원점 설정 (첫 GPS 위치)"""
        try:
            # GPS → UTM 변환
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

            rospy.loginfo("🎯 map 프레임 원점 설정 완료!")
            rospy.loginfo(f"   GPS: ({lat:.6f}, {lon:.6f})")
            rospy.loginfo(f"   UTM: ({easting:.2f}, {northing:.2f})")
            rospy.loginfo(f"   Zone: {zone_num}{zone_letter}")
            rospy.loginfo(f"   map 프레임 원점 = (0, 0)")

        except Exception as e:
            rospy.logerr(f"❌ map 원점 설정 실패: {e}")

    def gps_to_map(self, lat, lon):
        """GPS 좌표를 map 프레임 좌표로 변환"""
        if not self.origin_set:
            rospy.logwarn("⚠️  map 원점이 설정되지 않음")
            return None, None

        try:
            # GPS → UTM 변환
            easting, northing, _, _ = utm.from_latlon(lat, lon)

            # map 프레임 상대 좌표 계산
            map_x = easting - self.map_origin_utm['easting']
            map_y = northing - self.map_origin_utm['northing']

            return map_x, map_y

        except Exception as e:
            rospy.logerr(f"❌ GPS → map 변환 실패 ({lat}, {lon}): {e}")
            return None, None

    def waypoints_gps_callback(self, msg):
        """웹에서 받은 GPS 웨이포인트를 map 프레임으로 변환"""
        try:
            data = json.loads(msg.data)

            if 'waypoints' not in data:
                rospy.logwarn("⚠️  'waypoints' 키 없음")
                return

            if not self.origin_set:
                rospy.logwarn("❌ map 원점이 설정되지 않음. GPS 수신 대기 중...")
                return

            waypoints_gps = data['waypoints']
            rospy.loginfo(f"📥 GPS 웨이포인트 수신: {len(waypoints_gps)}개")

            # PoseArray 생성
            pose_array = PoseArray()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = "map"

            converted_count = 0

            for i, wp in enumerate(waypoints_gps):
                if 'lat' not in wp or 'lon' not in wp:
                    rospy.logwarn(f"⚠️  웨이포인트 {i}: GPS 좌표 누락")
                    continue

                # GPS → map 변환
                map_x, map_y = self.gps_to_map(wp['lat'], wp['lon'])

                if map_x is None or map_y is None:
                    continue

                # Pose 생성
                pose = Pose()
                pose.position.x = map_x
                pose.position.y = map_y
                pose.position.z = 0.0
                pose.orientation.w = 1.0  # 방향은 기본값

                pose_array.poses.append(pose)
                converted_count += 1

                # 처음 3개와 마지막 3개만 로깅
                if i < 3 or i >= len(waypoints_gps) - 3:
                    rospy.loginfo(f"   WP{i+1}: GPS({wp['lat']:.6f}, {wp['lon']:.6f}) → map({map_x:.1f}, {map_y:.1f})")
                elif i == 3 and len(waypoints_gps) > 6:
                    rospy.loginfo(f"   ... (중간 {len(waypoints_gps)-6}개 생략)")

            # PoseArray 발행
            if converted_count > 0:
                self.waypoints_pub.publish(pose_array)
                rospy.loginfo(f"✅ map 프레임 웨이포인트 발행: {converted_count}개")
            else:
                rospy.logwarn("❌ 변환된 웨이포인트 없음")

        except json.JSONDecodeError as e:
            rospy.logerr(f"❌ JSON 파싱 오류: {e}")
        except Exception as e:
            rospy.logerr(f"❌ 웨이포인트 처리 오류: {e}")

    def publish_status(self, event):
        """상태 정보 발행"""
        status = {
            'node': 'kakao_route_planner',
            'map_origin_set': self.origin_set,
            'gps_available': self.gps_available,
            'timestamp': rospy.Time.now().to_sec()
        }

        if self.origin_set:
            status['map_origin'] = {
                'lat': self.map_origin_utm['lat'],
                'lon': self.map_origin_utm['lon'],
                'utm_zone': f"{self.utm_zone_number}{self.utm_zone_letter}"
            }

        if self.current_gps:
            status['current_gps'] = self.current_gps

        self.status_pub.publish(json.dumps(status))


if __name__ == '__main__':
    try:
        node = KakaoRoutePlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Kakao Route Planner 종료")
