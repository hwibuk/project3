#!/usr/bin/env python3
import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = os.path.expanduser('~/cyclone_dds.xml')
os.environ['ROS_STATIC_PEERS'] = '192.168.0.102'

import socket
import json
import threading
import rclpy
from rclpy.node import Node
from mycobot_interfaces.msg import MycobotSetAngles


class PiSenderNode(Node):
    def __init__(self):
        super().__init__('pi_sender')
        self.pub = self.create_publisher(MycobotSetAngles, '/mycobot/angles_goal', 10)
        self.get_logger().info('[Pi 송신] 시작 (cyclonedds, 독립 프로세스)')

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


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 9999))
    sock.settimeout(1.0)

    rclpy.init()
    node = PiSenderNode()

    def recv_loop():
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(4096)
                payload = json.loads(data.decode())
                node.send_angles(payload['angles'], payload['speed'])
            except socket.timeout:
                continue
            except Exception as e:
                node.get_logger().error(f'오류: {e}')

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
    main()
