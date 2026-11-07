"""Full on-robot stack (Raspberry Pi 5): hardware drivers + the shared autonomy pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('autonom_bringup'), 'launch', launch_file])),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    use_imu = LaunchConfiguration('use_imu')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_imu', default_value='false', description='Start the IMU driver and fuse /imu/data in the EKF'),
        DeclareLaunchArgument(
            'use_rf2o', default_value='true', description='Run rf2o laser odometry and fuse it in the EKF'),
        DeclareLaunchArgument(
            'autonomous_loop', default_value='true', description='Run the loop patrol once Nav2 and the map are up'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0', description='RPLiDAR S3 serial port'),
        _include('hardware.launch.py', {
            'use_imu': use_imu,
            'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
        }),
        _include('autonomy.launch.py', {
            'use_sim_time': 'false',
            'use_imu': use_imu,
            'use_rf2o': LaunchConfiguration('use_rf2o'),
            'autonomous_loop': LaunchConfiguration('autonomous_loop'),
        }),
    ])
