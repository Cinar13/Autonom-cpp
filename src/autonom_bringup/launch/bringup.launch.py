"""Full on-robot stack (Raspberry Pi 5): hardware drivers + the shared autonomy pipeline.

Nav2 modes (same overlays as the simulation, tuned for this robot; measurements in docs/SESSION_2026-11-09.md):
  controller:=rpp    Regulated Pure Pursuit throughout (default)
  controller:=mppi   RPP for the mapping lap, MPPI for the race (config/nav2_mppi.yaml); check its CPU on the Pi
  planner:=navfn     NavFn (default) | smac (config/nav2_smac.yaml) | thetastar (config/nav2_thetastar.yaml)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = ('rpp', 'mppi')
PLANNERS = ('navfn', 'smac', 'thetastar')


def _include(launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('autonom_bringup'), 'launch', launch_file])),
        launch_arguments=arguments.items(),
    )


def _nav2_overlay(context):
    """Comma-separated Nav2 overlays for the chosen controller and planner ('' = base nav2.yaml only)."""
    config = os.path.join(get_package_share_directory('autonom_bringup'), 'config')
    controller = LaunchConfiguration('controller').perform(context)
    planner = LaunchConfiguration('planner').perform(context)
    if controller not in CONTROLLERS:
        raise ValueError(f"controller must be one of {CONTROLLERS}, got '{controller}'")
    if planner not in PLANNERS:
        raise ValueError(f"planner must be one of {PLANNERS}, got '{planner}'")
    overlays = []
    if controller != 'rpp':
        overlays.append(os.path.join(config, f'nav2_{controller}.yaml'))
    if planner != 'navfn':
        overlays.append(os.path.join(config, f'nav2_{planner}.yaml'))
    return ','.join(overlays)


def _autonomy(context):
    use_imu = LaunchConfiguration('use_imu')
    return [_include('autonomy.launch.py', {
        'use_sim_time': 'false',
        'use_imu': use_imu,
        'use_rf2o': LaunchConfiguration('use_rf2o'),
        'autonomous_loop': LaunchConfiguration('autonomous_loop'),
        'nav2_params_overlay': _nav2_overlay(context),
    })]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_imu', default_value='false', description='Start the IMU driver and fuse /imu/data in the EKF'),
        DeclareLaunchArgument(
            'use_rf2o', default_value='true', description='Run rf2o laser odometry and fuse it in the EKF'),
        DeclareLaunchArgument(
            'autonomous_loop', default_value='true', description='Run the loop patrol once Nav2 and the map are up'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0', description='RPLiDAR S3 serial port'),
        DeclareLaunchArgument(
            'controller', default_value='rpp',
            description='Path follower: rpp (Regulated Pure Pursuit) | mppi (RPP maps, MPPI races)'),
        DeclareLaunchArgument(
            'planner', default_value='navfn', description='Global planner: navfn | smac (Smac 2D) | thetastar (Theta*)'),
        _include('hardware.launch.py', {
            'use_imu': LaunchConfiguration('use_imu'),
            'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
        }),
        OpaqueFunction(function=_autonomy),
    ])
