#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class PoseTargetController(Node):
    def __init__(self):
        super().__init__('pose_target_controller')

        # MoveItPy 초기화
        self.moveit = MoveItPy(node_name="pose_target_controller")
        self.arm = self.moveit.get_planning_component("arm_group")

        self.get_logger().info("MoveItPy 초기화 완료")

        # /target_pose 토픽 구독
        self.sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_pose_callback,
            10
        )

        self.get_logger().info("'/target_pose' 토픽 구독 시작")

    def target_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"목표 좌표 수신: x={msg.pose.position.x:.3f}, "
            f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
        )

        # 시작 상태를 현재 상태로 설정
        self.arm.set_start_state_to_current_state()

        # 목표 자세 설정
        self.arm.set_goal_state(pose_stamped_msg=msg, pose_link="joint6_flange")

        # 경로 계획
        plan_result = self.arm.plan()

        if plan_result:
            self.get_logger().info("경로 계획 성공 → 실행")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            self.get_logger().info("실행 완료!")
        else:
            self.get_logger().warn("경로 계획 실패")


def main(args=None):
    rclpy.init(args=args)
    node = PoseTargetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
