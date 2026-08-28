from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('level', default_value='level1'),
        DeclareLaunchArgument('keyboard_control', default_value='false'),
        DeclareLaunchArgument('show_lidar', default_value='true'),
        Node(
            package='track_sim',
            executable='simulator',
            name='track_sim',
            output='screen',
            parameters=[{
                'level': LaunchConfiguration('level'),
                'keyboard_control': LaunchConfiguration('keyboard_control'),
                'show_lidar': LaunchConfiguration('show_lidar'),
            }],
        ),
    ])
