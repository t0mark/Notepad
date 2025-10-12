#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Waypoint Manager Node
- Kakao API의 /kakao/path를 구독하여 웨이포인트로 사용
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
        self.goal_timeout = rospy.get_param('~goal_timeout', 60.0)
        self.max_retries = rospy.get_param('~max_retries', 3)
        self.goal_tolerance = rospy.get_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.5)
        self.max_deviation = rospy.get_param('~max_deviation', 5.0)
        self.skip_unreachable = rospy.get_param('~skip_unreachable', True)
        self.check_waypoint_cost = rospy.get_param('~check_waypoint_cost', True)
        self.lethal_cost_threshold = rospy.get_param('/move_base/GlobalPlanner/lethal_cost', 253)
        self.global_costmap_size = rospy.get_param('/move_base/global_costmap/width', 100.0)
        self.split_distance = rospy.get_param('~split_distance', 15.0)

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

    def find_closest_forward_waypoint(self, start_idx=None):
        """
        현재 로봇 위치에서 가장 가까운 미래 웨이포인트 찾기
        (역방향 주행 방지)

        Args:
            start_idx: 검색 시작 인덱스 (None이면 current_waypoint_idx 사용)
        """
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return 0

        if start_idx is None:
            start_idx = self.current_waypoint_idx

        min_distance = float('inf')
        closest_idx = start_idx

        # 지정된 인덱스부터 끝까지만 검색
        for i in range(start_idx, len(self.waypoints)):
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
            return True, 0.0

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
            new_pose.pose.position.z = 0.0  # 지면 레벨로 고정

            # 방향은 기본값 (w=1.0)
            new_pose.pose.orientation.w = 1.0

            result.append(new_pose)

        return result

    def validate_and_split_waypoints(self, waypoints):
        """
        초기 웨이포인트 처리: 그대로 보존 (삭제 없음)
        거리 기반 분할도 일단 제거 (동적으로 처리)
        """
        if not waypoints:
            return []

        rospy.loginfo(f"📊 웨이포인트 수신: {len(waypoints)}개 (원본 보존)")

        return waypoints

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

            # pose를 복사하여 원본을 수정하지 않도록 함
            text_marker.pose.position.x = waypoint.pose.position.x
            text_marker.pose.position.y = waypoint.pose.position.y
            text_marker.pose.position.z = waypoint.pose.position.z + 1.0  # 텍스트를 구 위에 표시
            text_marker.pose.orientation = waypoint.pose.orientation

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

        # Path의 모든 포즈를 그대로 웨이포인트로 사용
        waypoints = list(path_msg.poses)

        if not waypoints:
            rospy.logwarn("❌ 빈 경로가 수신됨")
            return

        # z 좌표를 0으로 고정 (지면 레벨)
        for wp in waypoints:
            wp.pose.position.z = 0.0

        # 웨이포인트 검증 및 분할
        waypoints = self.validate_and_split_waypoints(waypoints)

        self.waypoints = waypoints

        # 쿼터니언 초기화: 경로에 방향 정보가 없는 경우를 대비
        for wp in self.waypoints:
            q = wp.pose.orientation
            if q.x == 0 and q.y == 0 and q.z == 0 and q.w == 0:
                q.w = 1.0

        if not self.waypoints:
            rospy.logwarn("❌ 검증 후 유효한 웨이포인트 없음")
            return

        # 새 경로이므로 0번부터 검색하여 가장 가까운 웨이포인트 찾기
        self.current_waypoint_idx = self.find_closest_forward_waypoint(start_idx=0)
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

        robot_pose = self.get_robot_pose()
        if not robot_pose:
            rospy.logerr("❌ 로봇 위치를 가져올 수 없습니다")
            return

        target_waypoint = self.waypoints[self.current_waypoint_idx]

        # 현재 목표가 global costmap 안에 있는지 체크
        if not self.is_waypoint_in_costmap(target_waypoint, robot_pose):
            rospy.logwarn(f"⚠️  웨이포인트 {self.current_waypoint_idx + 1}이 costmap 밖에 있습니다")

            # 이전 waypoint 위치 (없으면 현재 로봇 위치)
            if self.current_waypoint_idx > 0:
                prev_waypoint = self.waypoints[self.current_waypoint_idx - 1]
            else:
                prev_waypoint = robot_pose

            # 중간 지점 생성 (2등분)
            intermediate = self.split_waypoint(prev_waypoint, target_waypoint, num_splits=2)

            # intermediate[0]이 중간점, intermediate[1]이 원래 목표
            # 중간점을 현재 인덱스에 삽입
            self.waypoints.insert(self.current_waypoint_idx, intermediate[0])

            rospy.loginfo(f"➕ 중간 웨이포인트 추가: ({intermediate[0].pose.position.x:.2f}, {intermediate[0].pose.position.y:.2f})")
            rospy.loginfo(f"📊 총 웨이포인트: {len(self.waypoints)}개")

            # 마커 업데이트
            self.publish_waypoint_markers()

            # 이제 추가된 중간 waypoint가 목표가 됨
            target_waypoint = self.waypoints[self.current_waypoint_idx]

        goal = MoveBaseGoal()
        goal.target_pose = target_waypoint
        goal.target_pose.header.stamp = rospy.Time.now()

        # 방향 무시: goal의 orientation을 로봇의 현재 방향으로 설정
        goal.target_pose.pose.orientation = robot_pose.pose.orientation

        rospy.loginfo("─" * 60)
        rospy.loginfo(f"🚀 웨이포인트 전송 중 {self.current_waypoint_idx + 1}/{len(self.waypoints)}")
        rospy.loginfo(f"   위치: ({target_waypoint.pose.position.x:.2f}, {target_waypoint.pose.position.y:.2f})")
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
