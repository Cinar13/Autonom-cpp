"""robot_localization EKF fusing wheel + rf2o odometry, and the IMU when use_imu:=true."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _ekf_node(context):
    config_dir = PathJoinSubstitution([FindPackageShare('autonom_bringup'), 'config'])
    parameter_files = [PathJoinSubstitution([config_dir, 'ekf.yaml'])]
    if LaunchConfiguration('use_imu').perform(context).lower() in ('true', '1', 'yes'):
        parameter_files.append(PathJoinSubstitution([config_dir, 'ekf_imu.yaml']))

    return [Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=parameter_files,
        remappings=[('odometry/filtered', '/odometry/filtered')],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='false', description='Fuse /imu/data in the EKF'),
        OpaqueFunction(function=_ekf_node),
    ])
