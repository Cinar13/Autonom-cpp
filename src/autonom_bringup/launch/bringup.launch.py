"""Full on-robot stack (Raspberry Pi 5): drivers, EKF, slam_toolbox, Nav2 and the autonomous loop patrol."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    bringup_share = FindPackageShare('autonom_bringup')
    use_imu = LaunchConfiguration('use_imu')

    arguments = [
        DeclareLaunchArgument(
            'use_imu', default_value='false', description='Start the IMU driver and fuse /imu/data in the EKF'),
        DeclareLaunchArgument(
            'autonomous_loop', default_value='true', description='Run the loop patrol once Nav2 and the map are up'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0', description='RPLiDAR S3 serial port'),
        DeclareLaunchArgument(
            'nav2_startup_delay', default_value='5.0',
            description='Seconds to wait for TF (map->odom->base) before starting Nav2'),
    ]

    hardware = _include('autonom_bringup', 'hardware.launch.py', {
        'use_imu': use_imu,
        'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
    })
    localization = _include('autonom_bringup', 'localization.launch.py', {'use_imu': use_imu})
    slam = _include('slam_toolbox', 'online_async_launch.py', {
        'slam_params_file': PathJoinSubstitution([bringup_share, 'config', 'slam_toolbox.yaml']),
        'use_sim_time': 'false',
        'autostart': 'true',
    })
    navigation = TimerAction(
        period=LaunchConfiguration('nav2_startup_delay'),
        actions=[_include('autonom_bringup', 'navigation.launch.py', {})],
    )

    loop_patrol = Node(
        package='autonom_bringup',
        executable='loop_patrol.py',
        name='loop_patrol',
        parameters=[PathJoinSubstitution([bringup_share, 'config', 'loop_patrol.yaml'])],
        condition=IfCondition(LaunchConfiguration('autonomous_loop')),
        output='screen',
    )

    return LaunchDescription(arguments + [hardware, localization, slam, navigation, loop_patrol])
