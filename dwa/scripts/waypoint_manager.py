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
        self.check_waypoint_cost = rospy.get_param('~check_waypoint_cost', True)
        self.lethal_cost_threshold = rospy.get_param('~lethal_cost_threshold', 253)
        self.global_costmap_size = rospy.get_param('~global_costmap_size', 50.0)
        self.split_distance = rospy.get_param('~split_distance', 15.0)

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
        rospy.loginfo(f"  웨이포인트 코스트 체크: {self.check_waypoint_cost}")
        rospy.loginfo(f"  치명적 코스트 임계값: {self.lethal_cost_threshold}")
        rospy.loginfo(f"  Global costmap 크기: {self.global_costmap_size}m")
        rospy.loginfo(f"  분할 거리: {self.split_distance}m")
        rospy.loginfo("=" * 60)

        # State
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.retry_count = 0
        self.is_active = False
        self.latest_path = None
        self.goal_requested = False

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Action Client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("move_base 액션 서버 대기 중...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("✅ move_base 액션 서버 연결됨")

        # Service Clients
        rospy.loginfo("make_plan 서비스 대기 중...")
        rospy.wait_for_service('/move_base/make_plan', timeout=10.0)
        self.make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        rospy.loginfo("✅ make_plan 서비스 연결됨")

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
                # 중복점 추가 방지
                if self.calculate_distance(filtered[-1], current) > 0.1:
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

    def check_waypoint_reachable(self, waypoint):
        """
        웨이포인트가 도달 가능한지 체크 (make_plan 서비스 사용)
        Returns: (reachable: bool, plan_length: float)
        """
        if not self.check_waypoint_cost:
            return True, 0.0

        robot_pose = self.get_robot_pose()
        if not robot_pose:
            rospy.logwarn("⚠️  로봇 위치를 가져올 수 없어 waypoint 체크를 건너뜁니다")
            return True, 0.0

        try:
            # make_plan 서비스 호출
            req = GetPlan()
            req.start = robot_pose
            req.goal = waypoint
            req.tolerance = self.goal_tolerance

            resp = self.make_plan_service(req.start, req.goal, req.tolerance)

            if resp.plan and len(resp.plan.poses) > 0:
                # 경로 길이 계산
                plan_length = 0.0
                for i in range(1, len(resp.plan.poses)):
                    plan_length += self.calculate_distance(
                        resp.plan.poses[i-1], resp.plan.poses[i]
                    )
                return True, plan_length
            else:
                rospy.logwarn(f"⚠️  Waypoint ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})로의 경로를 찾을 수 없습니다")
                return False, 0.0

        except rospy.ServiceException as e:
            rospy.logwarn(f"⚠️  make_plan 서비스 호출 실패: {e}")
            return True, 0.0  # 서비스 실패 시 웨이포인트 유지

    def is_waypoint_in_costmap(self, waypoint, robot_pose):
        """
        웨이포인트가 global costmap 범위 안에 있는지 체크
        """
        if not robot_pose:
            return True

        # Rolling window의 경우 로봇 중심으로 costmap_size/2 반경
        max_range = self.global_costmap_size / 2.0

        dx = waypoint.pose.position.x - robot_pose.pose.position.x
        dy = waypoint.pose.position.y - robot_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        return distance < max_range

    def split_waypoint(self, start_pose, end_pose, num_splits=2):
        """
        두 웨이포인트 사이를 num_splits개로 분할
        Returns: List[PoseStamped] (중간 지점들, end_pose 포함)
        """
        result = []

        for i in range(1, num_splits + 1):
            t = i / num_splits  # 0 < t <= 1

            new_pose = PoseStamped()
            new_pose.header.frame_id = end_pose.header.frame_id
            new_pose.header.stamp = rospy.Time.now()

            # 선형 보간
            new_pose.pose.position.x = start_pose.pose.position.x + t * (end_pose.pose.position.x - start_pose.pose.position.x)
            new_pose.pose.position.y = start_pose.pose.position.y + t * (end_pose.pose.position.y - start_pose.pose.position.y)
            new_pose.pose.position.z = start_pose.pose.position.z + t * (end_pose.pose.position.z - start_pose.pose.position.z)

            # 방향은 end_pose를 향하도록
            angle = math.atan2(
                end_pose.pose.position.y - start_pose.pose.position.y,
                end_pose.pose.position.x - start_pose.pose.position.x
            )
            new_pose.pose.orientation.z = math.sin(angle / 2.0)
            new_pose.pose.orientation.w = math.cos(angle / 2.0)

            result.append(new_pose)

        return result

    def validate_and_split_waypoints(self, waypoints):
        """
        웨이포인트 검증 및 필요시 분할
        - 코스트가 높은 waypoint는 건너뛰기
        - Costmap 범위 밖의 waypoint는 중간 지점 추가
        """
        if not waypoints:
            return []

        robot_pose = self.get_robot_pose()
        validated = []
        skipped_count = 0

        i = 0
        while i < len(waypoints):
            wp = waypoints[i]

            # 1. 도달 가능성 체크
            reachable, _ = self.check_waypoint_reachable(wp)
            if not reachable:
                rospy.logwarn(f"⚠️  Waypoint {i+1} 건너뜀: 장애물 또는 높은 코스트")
                skipped_count += 1
                i += 1
                continue

            # 2. Costmap 범위 체크 및 분할
            if robot_pose and not self.is_waypoint_in_costmap(wp, robot_pose):
                # 현재 로봇 위치(또는 마지막 validated waypoint)에서 거리 계산
                start_pose = validated[-1] if validated else robot_pose
                distance = self.calculate_distance(start_pose, wp)

                if distance > self.split_distance:
                    # 분할 필요
                    num_splits = int(math.ceil(distance / self.split_distance))
                    rospy.loginfo(f"📏 Waypoint {i+1}이 {distance:.1f}m 떨어져 있어 {num_splits}개로 분할합니다")

                    split_poses = self.split_waypoint(start_pose, wp, num_splits)
                    validated.extend(split_poses)
                else:
                    validated.append(wp)
            else:
                validated.append(wp)

            i += 1

        if skipped_count > 0:
            rospy.loginfo(f"📊 웨이포인트 검증: {len(waypoints)}개 중 {skipped_count}개 건너뜀, {len(validated)}개 유효")

        return validated

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

            # 포즈 초기화
            line_marker.pose.orientation.w = 1.0

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
        waypoints = self.filter_waypoints(path_msg)

        # 웨이포인트 검증 및 분할 (새로운 기능!)
        waypoints = self.validate_and_split_waypoints(waypoints)

        # 각 웨이포인트의 방향을 다음 웨이포인트를 향하도록 수정
        for i in range(len(waypoints) - 1):
            current_wp_pos = waypoints[i].pose.position
            next_wp_pos = waypoints[i+1].pose.position

            angle = math.atan2(next_wp_pos.y - current_wp_pos.y, next_wp_pos.x - current_wp_pos.x)
            q = Quaternion(x=0, y=0, z=math.sin(angle / 2.0), w=math.cos(angle / 2.0))
            waypoints[i].pose.orientation = q

        # 마지막 웨이포인트는 이전 웨이포인트의 방향을 따름
        if len(waypoints) > 1:
            waypoints[-1].pose.orientation = waypoints[-2].pose.orientation

        self.waypoints = waypoints

        # 쿼터니언 초기화: 경로에 방향 정보가 없는 경우를 대비 (위에서 이미 설정했으므로 보험용)
        for wp in self.waypoints:
            q = wp.pose.orientation
            if q.x == 0 and q.y == 0 and q.z == 0 and q.w == 0:
                q.w = 1.0

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

        # 메인 루프에서 첫 웨이포인트를 전송하도록 요청
        self.goal_requested = True

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

            if self.is_active:
                rospy.loginfo("잠시 대기 후 다음 웨이포인트로 진행합니다...")
                rospy.sleep(3.0)  # Costmap 업데이트 시간 확보 (1.5s -> 3.0s)
                self.goal_requested = True  # 메인 루프에 다음 목표 요청

        elif status in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED]:
            # 실패 또는 취소됨: 재시도 또는 스킵
            if status == GoalStatus.PREEMPTED:
                rospy.logwarn("⚠️  목표가 취소되었습니다 (타임아웃 또는 새 경로 수신).")
            else:
                rospy.logwarn(f"⚠️  목표 도달에 실패했습니다: {status_text}")

            self.retry_count += 1

            if self.retry_count < self.max_retries:
                rospy.logwarn(f"   재시도를 수행합니다... ({self.retry_count}/{self.max_retries})")
                self.goal_requested = True  # 메인 루프에 재시도 요청
            else:
                if self.skip_unreachable:
                    rospy.logwarn(f"⚠️  도달 불가능한 웨이포인트 {self.current_waypoint_idx + 1}를 건너뜁니다.")
                    self.current_waypoint_idx += 1
                    self.retry_count = 0

                    # 마커 업데이트
                    self.publish_waypoint_markers()

                    if self.is_active:
                        self.goal_requested = True  # 메인 루프에 다음 목표 요청
                else:
                    rospy.logerr(f"❌ {self.max_retries}회 재시도 후에도 목표에 실패하여 주행을 중지합니다.")
                    self.is_active = False
        else:
            rospy.logwarn(f"⚠️  예상치 못한 목표 상태: {status_text}")

    def run(self):
        """메인 루프"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            if self.goal_requested and self.is_active:
                # 현재 action client가 활동 중이 아닐 때만 새 목표 전송
                client_state = self.move_base_client.get_state()
                if client_state not in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
                    self.goal_requested = False
                    self.send_next_waypoint()

            rate.sleep()


if __name__ == '__main__':
    try:
        manager = WaypointManager()
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("웨이포인트 관리자 종료됨")
