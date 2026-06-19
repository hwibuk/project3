#!/usr/bin/env python3
"""
go_to_xy_node.py

/goal_xy 토픽으로 (x, y) 좌표를 받으면, Nav2의 NavigateToPose 액션을 호출해서
장애물을 회피하며 그 좌표로 자율주행하게 하는 노드.

사용법:
    ros2 topic pub /goal_xy geometry_msgs/msg/Point "{x: 2.0, y: 1.5, z: 0.0}" --once

yaw(도착 시 바라볼 방향)는 기본적으로 0도(맵 +x 방향)로 설정.
필요하면 /goal_pose (PoseStamped) 토픽으로 방향까지 지정 가능.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class GoToXYNode(Node):

    def __init__(self):
        super().__init__('go_to_xy_node')

        self.declare_parameter('default_yaw', 0.0)  # 목표 도착 시 바라볼 방향(라디안)
        self.default_yaw = self.get_parameter('default_yaw').value

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 단순 (x,y)만 받는 토픽
        self.create_subscription(Point, '/goal_xy', self.goal_xy_cb, 10)

        # 방향까지 지정하고 싶을 때 쓰는 토픽 (선택사항)
        self.create_subscription(PoseStamped, '/goal_pose_xy', self.goal_pose_cb, 10)

        self._goal_handle = None

        self.get_logger().info(
            "go_to_xy_node 시작됨. "
            "사용법: ros2 topic pub /goal_xy geometry_msgs/msg/Point \"{x: 2.0, y: 1.5, z: 0.0}\" --once"
        )

    def goal_xy_cb(self, msg: Point):
        self.get_logger().info(f"좌표 수신: x={msg.x:.2f}, y={msg.y:.2f} -> Nav2로 전송")
        self.send_goal(msg.x, msg.y, self.default_yaw)

    def goal_pose_cb(self, msg: PoseStamped):
        # 이미 PoseStamped로 들어오면 그대로 사용
        self.get_logger().info(f"PoseStamped 좌표 수신 -> Nav2로 전송")
        self._send_pose_goal(msg)

    def send_goal(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # yaw -> quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._send_pose_goal(pose)

    def _send_pose_goal(self, pose: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("navigate_to_pose 액션 서버를 찾을 수 없습니다. Nav2가 실행 중인지 확인하세요.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"목표 전송: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("목표가 Nav2에 의해 거부되었습니다.")
            return

        self.get_logger().info("목표가 수락됨. 자율주행 시작.")
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = getattr(feedback, 'distance_remaining', None)
        if remaining is not None:
            self.get_logger().info(f"남은 거리: {remaining:.2f} m", throttle_duration_sec=2.0)

    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("목표 지점 도착 완료!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("이동이 취소되었습니다.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("이동이 중단되었습니다 (경로 계획 실패 또는 장애물로 막힘).")
        else:
            self.get_logger().warn(f"알 수 없는 종료 상태: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = GoToXYNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()