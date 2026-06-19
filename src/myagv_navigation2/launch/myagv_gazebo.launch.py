import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ── URDF 파일 경로 (install 경로 우선, 없으면 src 경로) ───
    urdf_install = os.path.join(
        get_package_share_directory('myagv_description'),
        'urdf',
        'myAGV_gazebo.urdf'
    )
    urdf_src = '/myagv_ros2/src/myagv_description/urdf/myAGV_gazebo.urdf'

    urdf_file = urdf_install if os.path.exists(urdf_install) else urdf_src

    assert os.path.exists(urdf_file), \
        f'URDF not found: {urdf_file}\n' \
        f'먼저 myAGV_gazebo.urdf 를 해당 경로에 복사하고 colcon build 하세요.'

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    assert len(robot_description.strip()) > 0, \
        f'URDF 파일이 비어 있습니다: {urdf_file}'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose      = LaunchConfiguration('x_pose',        default='0.0')
    y_pose      = LaunchConfiguration('y_pose',        default='0.0')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose',       default_value='0.0'),
        DeclareLaunchArgument('y_pose',       default_value='0.0'),

        # ── 1) Gazebo 서버 (world 없이 empty world) ────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': '/myagv_ros2/src/myagv_description/worlds/myagv_real_map.world', 'verbose': 'false'}.items(),
        ),

        # ── 2) Gazebo 클라이언트 GUI ───────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # ── 3) robot_state_publisher ───────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,          # LaunchConfiguration 아닌 bool 직접 지정
                'robot_description': robot_description,
            }],
        ),

        # ── 4) joint_state_publisher ───────────────────────────
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),

        # ── 5) Gazebo에 로봇 spawn (파일 직접 지정) ────────────
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_myagv',
            output='screen',
            arguments=[
                '-entity', 'myagv',
                '-file',  urdf_file,   # topic 대신 파일 경로 직접 사용
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.07',
            ],
        ),
    ])