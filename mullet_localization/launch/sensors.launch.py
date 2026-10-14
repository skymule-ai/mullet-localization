"""D435i and Mid-360 drivers under /sensors. Configs come from config_dir (the repo's config/)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config = PathJoinSubstitution([LaunchConfiguration('config_dir'), 'sensors'])
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value='/config'),
        DeclareLaunchArgument('d435i', default_value='true'),
        DeclareLaunchArgument('mid360', default_value='true'),
        # /sensors/d435i/{infra1,infra2}/image_rect_raw, /sensors/d435i/imu
        Node(
            package='realsense2_camera', executable='realsense2_camera_node',
            name='d435i', namespace='sensors', output='screen',
            parameters=[PathJoinSubstitution([config, 'd435i.yaml'])],
            condition=IfCondition(LaunchConfiguration('d435i'))),
        # /sensors/mid360/points, /sensors/mid360/imu
        Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='mid360', namespace='sensors', output='screen',
            parameters=[
                PathJoinSubstitution([config, 'mid360.yaml']),
                {'user_config_path': PathJoinSubstitution([config, 'MID360_config.json'])},
            ],
            remappings=[('livox/lidar', 'mid360/points'), ('livox/imu', 'mid360/imu')],
            condition=IfCondition(LaunchConfiguration('mid360'))),
    ])
