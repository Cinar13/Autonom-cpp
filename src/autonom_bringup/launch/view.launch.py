"""RViz2 for the WSL2 base station. Runs no robot nodes."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('autonom_bringup'), 'config', 'autonom.rviz'])],
            output='screen',
        ),
    ])
