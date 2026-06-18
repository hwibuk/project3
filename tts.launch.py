import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    # 사용자의 tts 패키지 절대 경로 설정
    tts_dir = os.path.expanduser('~/ros2_ws/src/tts')

    # [부품 1] Rosbridge Server 실행 (9091 포트 지정)
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={'port': '9091'}.items()
    )

    # [부품 2] Web Server 실행 (8000 포트, index.html이 있는 폴더 기준)
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd=tts_dir,
        output='screen'
    )

    # [부품 3] 통합 음성 및 주행 제어 노드 실행
    voice_nav_node = ExecuteProcess(
        cmd=['python3', 'tts_node.py'],
        cwd=tts_dir,
        output='screen'
    )

    # 3가지 프로세스를 한 번에 묶어서 반환
    return LaunchDescription([
        rosbridge_launch,
        web_server,
        voice_nav_node
    ])
