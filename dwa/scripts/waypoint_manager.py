#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Waypoint Manager Node
- Kakao API의 /kakao/path를 구독하여 적절한 간격의 웨이포인트 생성
- move_base ActionClient를 통해 순차적으로 웨이포인트 전송
- 장애물, costmap 경계, 타임아웃 등 다양한 예외 상황 처리
"""

import rospy
import actionlib
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.srv import GetPlan
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class WaypointManager:
    def __init__(self):
        rospy.init_node('waypoint_manager', anonymous=False)

        # Parameters
        self.min_waypoint_distance = rospy.get_param('~min_waypoint_distance', 3.0)
        self.max_waypoint_distance = rospy.get_param('~max_waypoint_distance', 10.0)
        self.angle_threshold = rospy.get_param('~angle_threshold', 0.5)  # rad
        self.goal_timeout = rospy.get_param('~goal_timeout', 60.0)
        self.max_retries = rospy.get_param('~max_retries', 3)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)
        self.max_deviation = rospy.get_param('~max_deviation', 5.0)
        self.skip_unreachable = rospy.get_param('~skip_unreachable', True)

        rospy.loginfo("=" * 60)
        rospy.loginfo("웨이포인트 관리자 파라미터:")
        rospy.loginfo(f"  최소 웨이포인트 거리: {self.min_waypoint_distance}m")
        rospy.loginfo(f"  최대 웨이포인트 거리: {self.max_waypoint_distance}m")
        rospy.loginfo(f"  각도 임계값: {self.angle_threshold} rad")
        rospy.loginfo(f"  목표 타임아웃: {self.goal_timeout}s")
        rospy.loginfo(f"  최대 재시도 횟수: {self.max_retries}")
        rospy.loginfo(f"  목표 허용 오차: {self.goal_tolerance}m")
        rospy.loginfo(f"  최대 편차: {self.max_deviation}m")
        rospy.loginfo(f"  도달 불가능 지점 건너뛰기: {self.skip_unreachable}")
        rospy.loginfo("=" * 60)

        # State
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.retry_count = 0
        self.is_active = False
        self.latest_path = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Action Client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("move_base 액션 서버 대기 중...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("✅ move_base 액션 서버 연결됨")

        # Subscribers
        rospy.Subscriber('/kakao/path', Path, self.path_callback, queue_size=1)

        # Publishers (for visualization)
        self.marker_pub = rospy.Publisher('/kakao/markers', MarkerArray, queue_size=1, latch=True)

        rospy.loginfo("✅ 웨이포인트 관리자 초기화 완료")
        rospy.loginfo("📡 /kakao/path 대기 중...")

    def calculate_distance(self, pose1, pose2):
        """두 pose 간의 유클리드 거리 계산"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def calculate_angle_diff(self, pose1, pose2, pose3):
        """
        세 점이 이루는 각도 변화 계산
        pose1 -> pose2 -> pose3
        """
        # Vector 1: pose1 -> pose2
        v1x = pose2.pose.position.x - pose1.pose.position.x
        v1y = pose2.pose.position.y - pose1.pose.position.y

        # Vector 2: pose2 -> pose3
        v2x = pose3.pose.position.x - pose2.pose.position.x
        v2y = pose3.pose.position.y - pose2.pose.position.y

        # 각도 계산
        angle1 = math.atan2(v1y, v1x)
        angle2 = math.atan2(v2y, v2x)

        # 각도 차이 (-pi ~ pi)
        diff = angle2 - angle1
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        return abs(diff)

    def filter_waypoints(self, path):
        """
        거리 + 각도 기반 웨이포인트 필터링
        - 최소 거리 이상 떨어진 지점만 선택
        - 큰 각도 변화가 있는 지점은 무조건 포함
        - 마지막 지점은 항상 포함
        """
        if not path.poses or len(path.poses) < 2:
            rospy.logwarn("빈 경로 또는 너무 짧은 경로가 수신됨")
            return []

        filtered = []
        filtered.append(path.poses[0])  # 첫 지점

        last_added = path.poses[0]

        for i in range(1, len(path.poses) - 1):
            current = path.poses[i]
            next_pose = path.poses[i + 1] if i + 1 < len(path.poses) else None

            # 거리 체크
            dist = self.calculate_distance(last_added, current)

            # 각도 변화 체크 (이전-현재-다음)
            angle_change = 0
            if len(filtered) >= 1 and next_pose:
                angle_change = self.calculate_angle_diff(filtered[-1], current, next_pose)

            # 추가 조건:
            # 1. 최소 거리 이상
            # 2. 또는 큰 각도 변화 (꺾이는 구간)
            # 3. 최대 거리 초과 시 무조건 추가
            if (dist >= self.min_waypoint_distance or
                angle_change >= self.angle_threshold or
                dist >= self.max_waypoint_distance):
                filtered.append(current)
                last_added = current

        # 마지막 지점 (목적지)
        if len(path.poses) > 0:
            last_dist = self.calculate_distance(last_added, path.poses[-1])
            if last_dist > 0.1:  # 중복 방지
                filtered.append(path.poses[-1])

        rospy.loginfo(f"📊 웨이포인트 필터링: {len(path.poses)} -> {len(filtered)} 개")

        return filtered

    def get_robot_pose(self):
        """현재 로봇 위치 가져오기 (map frame)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(1.0))

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"로봇 위치 가져오기 실패: {e}")
            return None

    def find_closest_forward_waypoint(self):
        """
        현재 로봇 위치에서 가장 가까운 미래 웨이포인트 찾기
        (역방향 주행 방지)
        """
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return 0

        min_distance = float('inf')
        closest_idx = self.current_waypoint_idx

        # 현재 인덱스부터 끝까지만 검색
        for i in range(self.current_waypoint_idx, len(self.waypoints)):
            dist = self.calculate_distance(robot_pose, self.waypoints[i])
            if dist < min_distance:
                min_distance = dist
                closest_idx = i

        return closest_idx

    def publish_waypoint_markers(self):
        """웨이포인트를 MarkerArray로 발행하여 시각화"""
        marker_array = MarkerArray()

        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pub.publish(marker_array)

        marker_array = MarkerArray()

        for i, waypoint in enumerate(self.waypoints):
            # 구 마커 (웨이포인트 위치)
            marker = Marker()
            marker.header.frame_id = waypoint.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose = waypoint.pose
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5

            # 색상 (현재 목표는 초록색, 나머지는 파란색, 완료된 것은 회색)
            if i < self.current_waypoint_idx:
                # 완료된 웨이포인트 - 회색
                marker.color = ColorRGBA(0.5, 0.5, 0.5, 0.8)
            elif i == self.current_waypoint_idx:
                # 현재 목표 - 초록색
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
            else:
                # 대기 중인 웨이포인트 - 파란색
                marker.color = ColorRGBA(0.0, 0.5, 1.0, 0.8)

            marker.lifetime = rospy.Duration(0)  # 영구 표시
            marker_array.markers.append(marker)

            # 텍스트 마커 (번호 표시)
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "waypoint_numbers"
            text_marker.id = i + 1000  # ID 충돌 방지
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose = waypoint.pose
            text_marker.pose.position.z += 1.0  # 텍스트를 구 위에 표시

            text_marker.scale.z = 0.5  # 텍스트 크기
            text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # 흰색
            text_marker.text = f"{i + 1}"
            text_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(text_marker)

        # 웨이포인트 간 연결선
        if len(self.waypoints) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = self.waypoints[0].header.frame_id
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "waypoint_path"
            line_marker.id = 10000
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            line_marker.scale.x = 0.1  # 선 두께
            line_marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.6)  # 노란색

            for waypoint in self.waypoints:
                line_marker.points.append(waypoint.pose.position)

            line_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(line_marker)

        self.marker_pub.publish(marker_array)
        rospy.loginfo(f"🎨 웨이포인트 마커 발행 완료: {len(self.waypoints)}개")

    def path_callback(self, path_msg):
        """새로운 경로 수신"""
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"📨 새로운 경로 수신: {len(path_msg.poses)}개 포즈")

        # 현재 진행 중인 goal 취소
        if self.is_active:
            rospy.loginfo("⚠️  현재 목표 취소 중...")
            self.move_base_client.cancel_all_goals()
            rospy.sleep(0.5)

        # 웨이포인트 필터링
        self.waypoints = self.filter_waypoints(path_msg)

        if not self.waypoints:
            rospy.logwarn("❌ 필터링 후 유효한 웨이포인트 없음")
            return

        # 가장 가까운 웨이포인트부터 시작
        self.current_waypoint_idx = self.find_closest_forward_waypoint()
        self.retry_count = 0
        self.is_active = True
        self.latest_path = path_msg

        # 마커 발행
        self.publish_waypoint_markers()

        rospy.loginfo(f"🎯 웨이포인트 {self.current_waypoint_idx + 1}/{len(self.waypoints)}부터 시작")
        rospy.loginfo("=" * 60)

        # 첫 번째 웨이포인트 전송
        self.send_next_waypoint()

    def send_next_waypoint(self):
        """다음 웨이포인트를 move_base로 전송"""
        if self.current_waypoint_idx >= len(self.waypoints):
            rospy.loginfo("=" * 60)
            rospy.loginfo("🎉 모든 웨이포인트 도달 완료!")
            rospy.loginfo("=" * 60)
            self.is_active = False
            return

        goal = MoveBaseGoal()
        goal.target_pose = self.waypoints[self.current_waypoint_idx]
        goal.target_pose.header.stamp = rospy.Time.now()

        wp = self.waypoints[self.current_waypoint_idx]
        rospy.loginfo("─" * 60)
        rospy.loginfo(f"🚀 웨이포인트 전송 중 {self.current_waypoint_idx + 1}/{len(self.waypoints)}")
        rospy.loginfo(f"   위치: ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})")
        rospy.loginfo(f"   재시도: {self.retry_count}/{self.max_retries}")

        # Goal 전송 (콜백과 타임아웃 설정)
        self.move_base_client.send_goal(
            goal,
            done_cb=self.goal_done_callback,
            feedback_cb=self.goal_feedback_callback
        )

        # 타임아웃 타이머 시작
        self.goal_start_time = rospy.Time.now()

    def goal_feedback_callback(self, feedback):
        """Goal 진행 중 피드백 (현재 위치 등)"""
        # 타임아웃 체크
        elapsed = (rospy.Time.now() - self.goal_start_time).to_sec()
        if elapsed > self.goal_timeout:
            rospy.logwarn(f"⏰ 목표 타임아웃 ({elapsed:.1f}초 > {self.goal_timeout}초)")
            self.move_base_client.cancel_goal()

    def goal_done_callback(self, status, result):
        """Goal 완료 콜백"""
        status_text = {
            GoalStatus.PENDING: 'PENDING',
            GoalStatus.ACTIVE: 'ACTIVE',
            GoalStatus.PREEMPTED: 'PREEMPTED',
            GoalStatus.SUCCEEDED: 'SUCCEEDED',
            GoalStatus.ABORTED: 'ABORTED',
            GoalStatus.REJECTED: 'REJECTED',
            GoalStatus.PREEMPTING: 'PREEMPTING',
            GoalStatus.RECALLING: 'RECALLING',
            GoalStatus.RECALLED: 'RECALLED',
            GoalStatus.LOST: 'LOST'
        }.get(status, 'UNKNOWN')

        rospy.loginfo(f"📍 목표 상태: {status_text}")

        if status == GoalStatus.SUCCEEDED:
            # 성공: 다음 웨이포인트로
            rospy.loginfo(f"✅ 웨이포인트 {self.current_waypoint_idx + 1} 도달!")
            self.current_waypoint_idx += 1
            self.retry_count = 0

            # 마커 업데이트
            self.publish_waypoint_markers()

            # 다음 웨이포인트 전송
            if self.is_active:
                rospy.sleep(0.5)  # 짧은 대기
                self.send_next_waypoint()

        elif status == GoalStatus.PREEMPTED:
            # 취소됨 (새 경로 수신 시)
            rospy.loginfo("⚠️  목표 취소됨 (새 경로 수신)")

        elif status in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
            # 실패: 재시도 또는 스킵
            self.retry_count += 1

            if self.retry_count < self.max_retries:
                rospy.logwarn(f"⚠️  목표 실패, 재시도 중... ({self.retry_count}/{self.max_retries})")
                rospy.sleep(1.0)
                self.send_next_waypoint()
            else:
                if self.skip_unreachable:
                    rospy.logwarn(f"⚠️  도달 불가능한 웨이포인트 {self.current_waypoint_idx + 1} 건너뛰기")
                    self.current_waypoint_idx += 1
                    self.retry_count = 0

                    # 마커 업데이트
                    self.publish_waypoint_markers()

                    if self.is_active:
                        rospy.sleep(0.5)
                        self.send_next_waypoint()
                else:
                    rospy.logerr(f"❌ {self.max_retries}회 재시도 후 목표 실패. 중지.")
                    self.is_active = False
        else:
            rospy.logwarn(f"⚠️  예상치 못한 목표 상태: {status_text}")

    def run(self):
        """메인 루프"""
        rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():
            # 주기적으로 로봇 상태 확인 (필요시)
            if self.is_active and self.waypoints:
                robot_pose = self.get_robot_pose()
                if robot_pose and self.current_waypoint_idx < len(self.waypoints):
                    # 현재 웨이포인트와의 거리 체크
                    dist = self.calculate_distance(
                        robot_pose,
                        self.waypoints[self.current_waypoint_idx]
                    )

                    # 디버그 정보 (10초마다)
                    if rospy.Time.now().to_sec() % 10 < 1:
                        rospy.logdebug(f"웨이포인트까지 거리: {dist:.2f}m")

            rate.sleep()


if __name__ == '__main__':
    try:
        manager = WaypointManager()
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("웨이포인트 관리자 종료됨")
