#!/usr/bin/env python3
import os
import sys
import math
import time
import socket
import threading
import json
import multiprocessing

# 모든 프로세스에서 ROS2 패키지 경로 공유
ROS_INSTALL_PATH = os.path.expanduser('~/ros2_ws/install')
PYTHON_VERSION = f'python{sys.version_info.major}.{sys.version_info.minor}'
SITE_PACKAGES = f'lib/{PYTHON_VERSION}/site-packages'

def setup_ros_paths():
    """서브프로세스에서 ROS2 패키지 찾을 수 있도록 경로 설정"""
    paths = [
        f'/opt/ros/jazzy/{SITE_PACKAGES}',
        f'{ROS_INSTALL_PATH}/mycobot_interfaces/{SITE_PACKAGES}',
        f'{ROS_INSTALL_PATH}/mycobot_communication/{SITE_PACKAGES}',
        f'{ROS_INSTALL_PATH}/mycobot_280_moveit2/{SITE_PACKAGES}',
    ]
    for p in paths:
        if p not in sys.path:
            sys.path.insert(0, p)

    # LD_LIBRARY_PATH도 설정
    lib_paths = [
        '/opt/ros/jazzy/lib',
        f'{ROS_INSTALL_PATH}/mycobot_interfaces/lib',
    ]
    existing = os.environ.get('LD_LIBRARY_PATH', '')
    os.environ['LD_LIBRARY_PATH'] = ':'.join(lib_paths) + ':' + existing


def process_moveit_receiver():
    setup_ros_paths()
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ.pop('CYCLONEDDS_URI', None)

    import rclpy
    from rclpy.node import Node
    from moveit_msgs.msg import DisplayTrajectory

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    class ReceiverNode(Node):
        def __init__(self):
            super().__init__('moveit_receiver')
            self.sub = self.create_subscription(
                DisplayTrajectory,
                '/display_planned_path',
                self.callback,
                10
            )
            self.get_logger().info('[A] MoveIt2 수신 노드 시작 (fastrtps)')

        def callback(self, msg):
            if not msg.trajectory:
                return
            traj = msg.trajectory[0].joint_trajectory
            points = traj.points
            if not points:
                return
            self.get_logger().info(f'[A] 궤적 수신: {len(points)}개 포인트')
            for i, point in enumerate(points):
                pos = list(point.positions)
                if len(pos) < 6:
                    continue
                data = {
                    'angles': [math.degrees(p) for p in pos[:6]],
                    'speed': 30,
                    'dt': (
                        points[i+1].time_from_start.sec +
                        points[i+1].time_from_start.nanosec * 1e-9 -
                        point.time_from_start.sec -
                        point.time_from_start.nanosec * 1e-9
                    ) if i < len(points)-1 else 0
                }
                sock.sendto(json.dumps(data).encode(), ('127.0.0.1', 9999))
                if 0 < data['dt'] < 1.0:
                    time.sleep(data['dt'])
            self.get_logger().info('[A] 전송 완료!')

    rclpy.init()
    node = ReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def process_pi_sender():
    setup_ros_paths()
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['CYCLONEDDS_URI'] = os.path.expanduser('~/cyclone_dds.xml')
    os.environ['ROS_STATIC_PEERS'] = '192.168.0.102'

    import rclpy
    from rclpy.node import Node
    from mycobot_interfaces.msg import MycobotSetAngles

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 9999))
    sock.settimeout(1.0)

    class SenderNode(Node):
        def __init__(self):
            super().__init__('pi_sender')
            self.pub = self.create_publisher(
                MycobotSetAngles,
                '/mycobot/angles_goal',
                10
            )
            self.get_logger().info('[B] Pi 송신 노드 시작 (cyclonedds)')

        def send_angles(self, angles, speed):
            msg = MycobotSetAngles()
            msg.joint_1 = float(angles[0])
            msg.joint_2 = float(angles[1])
            msg.joint_3 = float(angles[2])
            msg.joint_4 = float(angles[3])
            msg.joint_5 = float(angles[4])
            msg.joint_6 = float(angles[5])
            msg.speed   = int(speed)
            self.pub.publish(msg)
            self.get_logger().info(
                f'[B] → Pi: [{angles[0]:.1f}, {angles[1]:.1f}, '
                f'{angles[2]:.1f}, {angles[3]:.1f}, '
                f'{angles[4]:.1f}, {angles[5]:.1f}]'
            )

    rclpy.init()
    node = SenderNode()

    def recv_loop():
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(4096)
                payload = json.loads(data.decode())
                node.send_angles(payload['angles'], payload['speed'])
            except socket.timeout:
                continue
            except Exception as e:
                print(f'[B] 오류: {e}')

    t = threading.Thread(target=recv_loop, daemon=True)
    t.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    print('🚀 MoveIt2 ↔ myCobot 브릿지 시작')
    print('  프로세스 A: MoveIt2 수신 (fastrtps)')
    print('  프로세스 B: Pi 송신 (cyclonedds)')

    pB = multiprocessing.Process(target=process_pi_sender, name='PiSender')
    pB.start()
    time.sleep(2)
    pA = multiprocessing.Process(target=process_moveit_receiver, name='MoveitReceiver')
    pA.start()

    try:
        pA.join()
        pB.join()
    except KeyboardInterrupt:
        print('\n종료 중...')
        pA.terminate()
        pB.terminate()
