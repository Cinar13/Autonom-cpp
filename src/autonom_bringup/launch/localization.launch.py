"""robot_localization EKF: wheel odometry, plus rf2o (use_rf2o:=true) and the IMU (use_imu:=true)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from autonom_bringup.launch_utils import is_true


def _ekf_node(context):
    config_dir = PathJoinSubstitution([FindPackageShare('autonom_bringup'), 'config'])
    parameters = [PathJoinSubstitution([config_dir, 'ekf.yaml'])]
    if is_true(context, 'use_rf2o'):
        parameters.append(PathJoinSubstitution([config_dir, 'ekf_rf2o.yaml']))
    if is_true(context, 'use_imu'):
        parameters.append(PathJoinSubstitution([config_dir, 'ekf_imu.yaml']))
    parameters.append({'use_sim_time': LaunchConfiguration('use_sim_time')})

    return [Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=parameters,
        remappings=[('odometry/filtered', '/odometry/filtered')],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rf2o', default_value='true', description='Fuse /rf2o/odom in the EKF'),
        DeclareLaunchArgument('use_imu', default_value='false', description='Fuse /imu/data in the EKF'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock (Gazebo)'),
        OpaqueFunction(function=_ekf_node),
    ])
