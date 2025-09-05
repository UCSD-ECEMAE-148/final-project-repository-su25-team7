from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='team7_lidar',
            executable='team7_lidar_executable',
            output='screen',
            emulate_tty=True),
    ])
