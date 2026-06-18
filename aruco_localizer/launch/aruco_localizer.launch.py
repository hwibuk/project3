import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_layout = os.path.join(
        get_package_share_directory('aruco_localizer'),
        'config',
        'marker_layout.json'
    )

    marker_layout_file = LaunchConfiguration('marker_layout_file', default=default_layout)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('marker_layout_file', default_value=default_layout),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='aruco_localizer',
            executable='aruco_localizer_node',
            name='aruco_localizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'marker_layout_file': marker_layout_file,
                'marker_size': 0.20,
                'camera_topic': '/front_camera/image_raw',
                'camera_info_topic': '/front_camera/front_camera/camera_info',
                'publish_initialpose': True,
                'aruco_dict': 'DICT_4X4_50',
                'cam_x_in_base': 0.16,
                'cam_y_in_base': 0.0,
                'cam_z_in_base': 0.10,
                'cam_roll_in_base': 0.0,
                'cam_pitch_in_base': 0.0,
                'cam_yaw_in_base': 0.0,
            }],
        ),
    ])
