import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # slam_toolbox 파라미터 파일 (패키지 기본값 사용)
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(
            get_package_share_directory('slam_toolbox'),
            'config',
            'mapper_params_online_async.yaml'
        )
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_file,
            description='slam_toolbox 파라미터 yaml 경로'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/scan', '/scan'),   # lidar topic 이름이 다르면 여기서 수정
            ],
        ),
    ])