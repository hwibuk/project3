from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    # 1. MoveIt 기본 설정 파일 빌드
    moveit_config = (
        MoveItConfigsBuilder("firefighter", package_name="mycobot_280_moveit2")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 2. 딕셔너리로 변환 (여기서 변수를 명확히 정의합니다)
    moveit_config_dict = moveit_config.to_dict()

    # 3. MoveItPy가 요구하는 필수 파이프라인 및 계획 요청 파라미터 강제 주입
    moveit_config_dict.update({
        "planning_pipelines": {
            "pipeline_names": ["ompl"]
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",        # OMPL 플래너 지정
            "max_velocity_scaling_factor": 1.0, # 최대 속도
            "max_acceleration_scaling_factor": 1.0, # 최대 가속도
            "planning_time": 5.0,               # 최대 허용 계획 시간(초)
        }
    })

    # 4. robot_state_publisher 런치 파일 포함
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mycobot_280_moveit2"),
                "launch", "rsp.launch.py"
            )
        )
    )

    # 5. 컨트롤러 스포너 런치 파일 포함
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mycobot_280_moveit2"),
                "launch", "spawn_controllers.launch.py"
            )
        )
    )

    # 6. 우리가 작성한 파이썬 제어 노드 실행 (수정된 파라미터 딕셔너리 주입)
    pose_target_node = Node(
        package="pick_place_pkg",
        executable="pose_target_controller",
        name="pose_target_controller",
        output="screen",
        parameters=[
            moveit_config_dict,
        ],
    )

    # 기존 코드 ...
    return LaunchDescription([
        rsp_launch,
        controllers_launch,
        # 🔥 추가: 로봇의 상태를 표준으로 변환해주는 노드
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{'source_list': ['/mycobot/joint_states']}], # 이 부분은 실제 토픽 이름 확인 필요
        ),
        pose_target_node,
    ])