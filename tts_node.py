import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import os
import time

class RobotVoiceNode(Node):
    def __init__(self):
        super().__init__('robot_voice_node')
        
        # '/nav_voice_command' 토픽 구독
        self.subscription = self.create_subscription(
            String, '/nav_voice_command', self.listener_callback, 10)
        
        # 오디오 모듈 초기화
        pygame.mixer.init()
        
        # 코드가 실행되는 현재 폴더 경로
        self.audio_path = os.getcwd()
        self.get_logger().info(f'🔊 음성 재생 노드가 켜졌습니다. (폴더 위치: {self.audio_path})')

    def play_file(self, filename):
        """개별 파일 재생 함수"""
        file_path = os.path.join(self.audio_path, filename)
        if os.path.exists(file_path):
            self.get_logger().info(f'▶️ 재생 중: {filename}')
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                time.sleep(0.05)
        else:
            self.get_logger().warn(f'❌ 파일을 찾을 수 없습니다: {file_path}')

    def listener_callback(self, msg):
        location = msg.data.strip()
        self.get_logger().info(f'📥 웹으로부터 명령 수신: [{location}]')
        
        # 1단계: "구역1.mp3" -> "안내시작.mp3" (로 안내를 시작합니다)
        self.get_logger().info(f'→ 출발 안내 시작')
        self.play_file(f"{location}.mp3")
        self.play_file("안내시작.mp3")
        
        # 잠시 쉬고 (실제 로봇 이동 시간 대용, 테스트용 2초 대기)
        time.sleep(2.0)
        
        # 2단계: "구역1.mp3" -> "도착알림.mp3" (에 도착했습니다)
        self.get_logger().info(f'→ 도착 안내 시작')
        self.play_file(f"{location}.mp3")
        self.play_file("도착알림.mp3")

def main(args=None):
    rclpy.init(args=args)
    node = RobotVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드가 종료되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
