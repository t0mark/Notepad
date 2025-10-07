#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Kakao Route Planner Node

카카오 API에서 받은 GPS 웨이포인트를 그대로 발행하는 노드

입력:
  - /kakao/goal (String): 웹에서 받은 GPS 웨이포인트 (JSON)

출력:
  - /kakao/waypoints (GeoPath): GPS 웨이포인트 배열
  - /kakao/status (String): 경로 상태 정보
"""

import rospy
import json
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geographic_msgs.msg import GeoPath, GeoPoseStamped, GeoPoint


class KakaoRoutePlanner:
    def __init__(self):
        rospy.init_node('kakao_route_planner', anonymous=True)

        # GPS 상태
        self.current_gps = None
        self.gps_available = False
        self.waypoint_count = 0

        # Publishers (latch=True로 마지막 메시지 유지)
        self.waypoints_gps_pub = rospy.Publisher('/kakao/waypoints', GeoPath, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/kakao/status', String, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/kakao/goal', String, self.waypoints_gps_callback)
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)

        # Timer for status publishing
        rospy.Timer(rospy.Duration(5.0), self.publish_status)

        rospy.loginfo("🗺️  Kakao Route Planner 시작")
        rospy.loginfo("📡 GPS 웨이포인트를 받아서 발행합니다")

    def gps_callback(self, msg):
        """GPS 데이터 수신"""
        if msg.status.status < 0:
            return

        self.current_gps = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }
        self.gps_available = True

    def waypoints_gps_callback(self, msg):
        """웹에서 받은 GPS 웨이포인트를 GeoPath로 발행"""
        try:
            data = json.loads(msg.data)

            if 'waypoints' not in data:
                rospy.logwarn("⚠️  'waypoints' 키 없음")
                return

            waypoints_gps = data['waypoints']
            rospy.loginfo(f"📥 GPS 웨이포인트 수신: {len(waypoints_gps)}개")

            # GeoPath 생성
            geo_path = GeoPath()
            geo_path.header.stamp = rospy.Time.now()
            geo_path.header.frame_id = "map"

            valid_count = 0

            for i, wp in enumerate(waypoints_gps):
                if 'lat' not in wp or 'lon' not in wp:
                    rospy.logwarn(f"⚠️  웨이포인트 {i}: GPS 좌표 누락")
                    continue

                # GeoPoseStamped 생성
                geo_pose = GeoPoseStamped()
                geo_pose.header.stamp = rospy.Time.now()
                geo_pose.header.frame_id = "map"

                # GeoPoint 설정
                geo_pose.pose.position.latitude = wp['lat']
                geo_pose.pose.position.longitude = wp['lon']
                geo_pose.pose.position.altitude = wp.get('alt', 0.0)

                # Orientation (기본값)
                geo_pose.pose.orientation.x = 0.0
                geo_pose.pose.orientation.y = 0.0
                geo_pose.pose.orientation.z = 0.0
                geo_pose.pose.orientation.w = 1.0

                geo_path.poses.append(geo_pose)
                valid_count += 1

                # 처음 3개와 마지막 3개만 로깅
                if i < 3 or i >= len(waypoints_gps) - 3:
                    rospy.loginfo(f"   WP{i+1}: GPS({wp['lat']:.6f}, {wp['lon']:.6f})")
                elif i == 3 and len(waypoints_gps) > 6:
                    rospy.loginfo(f"   ... (중간 {len(waypoints_gps)-6}개 생략)")

            # GeoPath 발행
            if valid_count > 0:
                self.waypoints_gps_pub.publish(geo_path)
                self.waypoint_count = valid_count
                rospy.loginfo(f"✅ GPS 웨이포인트 발행: {valid_count}개")
            else:
                rospy.logwarn("❌ 유효한 웨이포인트 없음")

        except json.JSONDecodeError as e:
            rospy.logerr(f"❌ JSON 파싱 오류: {e}")
        except Exception as e:
            rospy.logerr(f"❌ 웨이포인트 처리 오류: {e}")

    def publish_status(self, event):
        """상태 정보 발행"""
        status = {
            'node': 'kakao_route_planner',
            'gps_available': self.gps_available,
            'waypoint_count': self.waypoint_count,
            'timestamp': rospy.Time.now().to_sec()
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
