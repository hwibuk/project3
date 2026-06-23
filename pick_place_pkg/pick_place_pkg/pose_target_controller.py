#!/usr/bin/env python3
#!/usr/bin/env python3
import os
import math
import time
import socket
import json
import subprocess
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy



# ──────────────────────────────────────
# Pi 송신 프로세스 (cyclonedds, 별도 RMW)
# ──────────────────────────────────────
def process_pi_sender():
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['CYCLONEDDS_URI'] = os.path.expanduser('~/cyclone_dds.xml')
    os.environ['ROS_STATIC_PEERS'] = '192.168.0.102'

    import rclpy as rclpy_inner
    from rclpy.node import Node as Node_inner
    from mycobot_interfaces.msg import MycobotSetAngles

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 9999))
    sock.settimeout(1.0)

    class SenderNode(Node_inner):
        def __init__(self):
            super().__init__('pi_sender')
            self.pub = self.create_publisher(
                MycobotSetAngles, '/mycobot/angles_goal', 10)
            self.get_logger().info('[Pi 송신] 시작 (cyclonedds)')

        def send_angles(self, angles, speed):
            msg = MycobotSetAngles()
            msg.joint_1 = float(angles[0])
            msg.joint_2 = float(angles[1])
            msg.joint_3 = float(angles[2])
            msg.joint_4 = float(angles[3])
            msg.joint_5 = float(angles[4])
            msg.joint_6 = float(angles[5])
            msg.speed = int(speed)
            self.pub.publish(msg)

    rclpy_inner.init()
    node = SenderNode()

    def recv_loop():
        while rclpy_inner.ok():
            try:
                data, _ = sock.recvfrom(4096)
                payload = json.loads(data.decode())
                node.send_angles(payload['angles'], payload['speed'])
            except socket.timeout:
                continue
            except Exception as e:
                print(f'[Pi 송신] 오류: {e}')

    t = threading.Thread(target=recv_loop, daemon=True)
    t.start()
    try:
        rclpy_inner.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy_inner.shutdown()


# ──────────────────────────────────────
# PC 메인 노드 (fastrtps, moveit_py)
# ──────────────────────────────────────
class PoseTargetController(Node):
    def __init__(self):
        super().__init__('pose_target_controller')

        self.moveit = MoveItPy(node_name="pose_target_controller")
        self.arm = self.moveit.get_planning_component("arm_group")

        # Pi로 보낼 소켓 (UDP, 로컬)
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info("MoveItPy 초기화 완료")

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

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=msg, pose_link="joint6_flange")

        plan_result = self.arm.plan()

        if not plan_result:
            self.get_logger().warn("경로 계획 실패")
            return

        self.get_logger().info("경로 계획 성공 → Pi 전송 + 시뮬레이션 실행")

        # 1) 시뮬레이션(RViz/ros2_control)에도 동일 trajectory 실행
        robot_trajectory = plan_result.trajectory
        self.moveit.execute(robot_trajectory, controllers=[])

        # 2) 같은 trajectory를 Pi로 순서대로 전송
        joint_traj = robot_trajectory.get_robot_trajectory_msg().joint_trajectory
        points = joint_traj.points

        if not points:
            self.get_logger().warn("trajectory에 포인트가 없습니다")
            return

        self.get_logger().info(f"Pi로 전송할 포인트 수: {len(points)}")

        for i, point in enumerate(points):
            positions = list(point.positions)
            if len(positions) < 6:
                continue

            data = {
                'angles': [math.degrees(p) for p in positions[:6]],
                'speed': 30,
            }
            self.send_sock.sendto(
                json.dumps(data).encode(), ('127.0.0.1', 9999)
            )

            if i < len(points) - 1:
                t_now = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                t_next_point = points[i + 1].time_from_start
                t_next = t_next_point.sec + t_next_point.nanosec * 1e-9
                dt = t_next - t_now
                if 0 < dt < 1.0:
                    time.sleep(dt)

        self.get_logger().info("Pi 전송 완료!")


def main(args=None):
    # Pi 송신 노드를 완전히 독립된 프로세스로 실행 (fork 아닌 exec)
    sender_script = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'pi_sender_node.py'
    )
    sender_env = os.environ.copy()
    sender_proc = subprocess.Popen(
        [sys.executable, sender_script],
        env=sender_env
    )
    time.sleep(2)

    rclpy.init(args=args)
    node = PoseTargetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sender_proc.terminate()


if __name__ == '__main__':
    main()