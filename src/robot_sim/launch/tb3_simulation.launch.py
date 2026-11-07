"""Gazebo Harmonic simulation of the full autonomy stack (use_sim_time:=true everywhere).

Swap the placeholder TurtleBot3 Burger for the custom robot with a single argument, e.g.
  robot_urdf:=$(ros2 pkg prefix autonom_description)/share/autonom_description/urdf/autonom.urdf.xacro
The description is expanded with `sim:=true`, so it must include autonom_description/urdf/gz_plugins.xacro.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from autonom_bringup.launch_utils import is_true

SIM_TIME = {'use_sim_time': True}


def _gz_sim(gz_args, on_exit_shutdown):
    # Scoped so the server's and GUI's gz_args do not overwrite each other.
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ' '.join(gz_args), 'on_exit_shutdown': on_exit_shutdown}.items(),
    )])


def _gazebo(context):
    world = LaunchConfiguration('world').perform(context)
    if not is_true(context, 'gui'):
        return [_gz_sim(['-s', '--headless-rendering', '-r', '-v', '2', world], 'true')]
    # Separate server and GUI processes: a combined `gz sim` aborts creating the Qt Quick context on
    # WSL2's GPU (d3d12) driver, while each works on its own. Closing the GUI leaves the simulation running.
    return [_gz_sim(['-s', '-r', '-v', '2', world], 'true'), _gz_sim(['-g', '-v', '2'], 'false')]


def generate_launch_description():
    sim_share = get_package_share_directory('robot_sim')
    bringup_share = get_package_share_directory('autonom_bringup')
    # package:// mesh URIs resolve against the parents of each description package's share directory.
    mesh_roots = {
        os.path.dirname(get_package_share_directory(package))
        for package in ('turtlebot3_description', 'autonom_description')
    }

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('robot_urdf'), ' sim:=true']), value_type=str)

    arguments = [
        DeclareLaunchArgument(
            'robot_urdf',
            default_value=os.path.join(sim_share, 'urdf', 'turtlebot3_burger_sim.urdf.xacro'),
            description='Robot URDF/xacro; expanded with sim:=true and used for both TF and the Gazebo spawn'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sim_share, 'worlds', 'circuit2_harmonic.sdf'),
            description='Gazebo world SDF'),
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo GUI (false: headless server)'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('use_imu', default_value='false', description='Fuse the simulated IMU in the EKF'),
        DeclareLaunchArgument(
            'use_rf2o', default_value='false',
            description='Run rf2o laser odometry and fuse it in the EKF (off by default to save CPU)'),
        DeclareLaunchArgument(
            'autonomous_loop', default_value='true', description='Run the loop patrol once Nav2 and the map are up'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Send the race start signal on launch (false: call /race/start yourself)'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='-4.7', description='Spawn y [m] (on the circuit2 track)'),
        DeclareLaunchArgument('yaw', default_value='-1.5708', description='Spawn yaw [rad]'),
        DeclareLaunchArgument(
            'slam_params_overlay', default_value='',
            description='Optional overlay on autonom_bringup slam_toolbox.yaml (sim LiDAR matches the S3)'),
        DeclareLaunchArgument(
            'nav2_params_overlay', default_value=os.path.join(sim_share, 'config', 'nav2_sim.yaml'),
            description='Sim overlay on autonom_bringup nav2.yaml / loop_patrol.yaml'),
    ]

    environment = [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_share, 'models')),
        *[AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', root) for root in sorted(mesh_roots)],
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, SIM_TIME],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', '-name', 'robot', '-z', '0.01',
            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': os.path.join(sim_share, 'config', 'bridge.yaml')}, SIM_TIME],
        output='screen',
    )

    # Same EKF input contract as the real robot: wheel odometry with covariance on /wheel/odom.
    wheel_odom_relay = Node(
        package='autonom_localization',
        executable='odometry_covariance_relay',
        name='wheel_odom_covariance_relay',
        parameters=[os.path.join(bringup_share, 'config', 'odometry_relays.yaml'), SIM_TIME],
        remappings=[('odom_in', '/odom'), ('odom_out', '/wheel/odom')],
        output='screen',
    )

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'autonomy.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'use_imu': LaunchConfiguration('use_imu'),
            'use_rf2o': LaunchConfiguration('use_rf2o'),
            'autonomous_loop': LaunchConfiguration('autonomous_loop'),
            'auto_start': LaunchConfiguration('auto_start'),
            'slam_params_overlay': LaunchConfiguration('slam_params_overlay'),
            'nav2_params_overlay': LaunchConfiguration('nav2_params_overlay'),
            'nav2_startup_delay': '10.0',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bringup_share, 'config', 'autonom.rviz')],
        parameters=[SIM_TIME],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription(arguments + environment + [
        OpaqueFunction(function=_gazebo),
        robot_state_publisher,
        spawn_robot,
        bridge,
        wheel_odom_relay,
        autonomy,
        rviz,
    ])
