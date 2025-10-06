#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Waypoint to Global Goal Converter
/global_waypoints (PoseArray) → /move_base_simple/goal (PoseStamped) 순차 전송
"""

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID


class WaypointToGlobalGoal:
    def __init__(self):
        rospy.init_node('waypoint_to_globalGoal', anonymous=True)

        # 웨이포인트 관리
        self.waypoints = []
        self.current_index = 0
        self.is_navigating = False
        self.goal_sent = False

        # 타임아웃 관리
        self.goal_start_time = None
        self.goal_timeout = 120.0  # 2분
        self.consecutive_fails = 0

        # Publishers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # Subscribers
        rospy.Subscriber('/global_waypoints', PoseArray, self.waypoints_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        # Timer
        rospy.Timer(rospy.Duration(1.0), self.check_timeout)

        rospy.loginfo("Waypoint to Goal Converter 시작")

    def waypoints_callback(self, msg):
        """웨이포인트 수신"""
        if not msg.poses:
            return

        # 기존 네비게이션 중단
        if self.is_navigating:
            self.cancel_pub.publish(GoalID())
            rospy.sleep(0.2)

        self.waypoints = msg.poses
        self.current_index = 0
        self.consecutive_fails = 0
        self.is_navigating = True

        rospy.loginfo(f"웨이포인트 수신: {len(self.waypoints)}개")
        self.send_next_goal()

    def send_next_goal(self):
        """다음 goal 전송"""
        if self.current_index >= len(self.waypoints):
            rospy.loginfo("네비게이션 완료!")
            self.is_navigating = False
            return

        if self.goal_sent:
            return

        waypoint = self.waypoints[self.current_index]

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose = waypoint

        self.goal_pub.publish(goal)
        self.goal_sent = True
        self.goal_start_time = rospy.Time.now()

        rospy.loginfo(f"Goal 전송: {self.current_index + 1}/{len(self.waypoints)}")

    def status_callback(self, msg):
        """move_base 상태 모니터링"""
        if not msg.status_list or not self.goal_sent:
            return

        status = msg.status_list[-1].status

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"WP{self.current_index + 1} 완료")
            self.consecutive_fails = 0
            self.next_waypoint()

        elif status in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
            rospy.logwarn(f"WP{self.current_index + 1} 실패")
            self.handle_failure()

    def check_timeout(self, _):
        """타임아웃 체크"""
        if not self.is_navigating or not self.goal_sent or not self.goal_start_time:
            return

        elapsed = (rospy.Time.now() - self.goal_start_time).to_sec()
        if elapsed > self.goal_timeout:
            rospy.logwarn(f"WP{self.current_index + 1} 타임아웃")
            self.handle_failure()

    def handle_failure(self):
        """실패 처리"""
        self.consecutive_fails += 1

        if self.consecutive_fails >= 3:
            rospy.logerr("연속 실패 3회 - 중단")
            self.is_navigating = False
            return

        self.next_waypoint()

    def next_waypoint(self):
        """다음 웨이포인트로 이동"""
        self.current_index += 1
        self.goal_sent = False
        self.goal_start_time = None

        rospy.sleep(0.3)
        self.send_next_goal()


if __name__ == '__main__':
    try:
        converter = WaypointToGlobalGoal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
