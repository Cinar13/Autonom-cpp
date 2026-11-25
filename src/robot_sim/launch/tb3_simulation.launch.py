"""Gazebo Harmonic simulation of the full autonomy stack (use_sim_time:=true everywhere).

The two halves can run as separate launches (scripts/start_sim.sh, then scripts/start_robot.sh --sim):
  use_autonomy:=false   the simulated world only: Gazebo, robot spawn, robot_state_publisher, the ros_gz bridge, the
                        sim stand-ins for the hardware drivers (wheel odometry, IMU relay) and RViz
  use_world:=false      the robot code only: autonomy.launch.py (EKF, slam_toolbox, Nav2, loop patrol, start gate)

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
    if not is_true(context, 'use_world'):
        return []
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
        Command(['xacro ', LaunchConfiguration('robot_urdf'), ' sim:=true ', LaunchConfiguration('robot_xacro_args')]),
        value_type=str)

    arguments = [
        DeclareLaunchArgument(
            'robot_xacro_args', default_value='',
            description='Extra xacro arguments for robot_urdf, e.g. "gz_max_velocity:=1.0" (Gazebo DiffDrive limits)'),
        DeclareLaunchArgument(
            'robot_urdf',
            default_value=os.path.join(sim_share, 'urdf', 'turtlebot3_burger_sim.urdf.xacro'),
            description='Robot URDF/xacro; expanded with sim:=true and used for both TF and the Gazebo spawn'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sim_share, 'worlds', 'circuit2_harmonic.sdf'),
            description='Gazebo world SDF'),
        DeclareLaunchArgument('use_world', default_value='true', description='Start the simulated world and RViz'),
        DeclareLaunchArgument(
            'use_autonomy', default_value='true', description='Start the robot code (autonomy.launch.py)'),
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
        DeclareLaunchArgument('wheel_radius', default_value='0.033', description='Sim robot wheel radius [m]'),
        DeclareLaunchArgument('wheel_separation', default_value='0.160', description='Sim robot wheel separation [m]'),
        DeclareLaunchArgument('left_wheel_joint', default_value='wheel_left_joint', description='Left wheel joint'),
        DeclareLaunchArgument('right_wheel_joint', default_value='wheel_right_joint', description='Right wheel joint'),
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

    # Same EKF input contract as the real robot: /wheel/odom from encoder counts. The simulated wheel joint angles
    # are counted with the real encoders' resolution (autonom_hardware hardware.yaml ticks_per_revolution: 28 CPR x
    # 50:1 = 1400) and integrated exactly like the robot's wheel_odometry node, with its covariances.
    wheel_odometry = Node(
        package='autonom_hardware',
        executable='sim_wheel_odometry_node',
        name='wheel_odometry',
        parameters=[
            os.path.join(get_package_share_directory('autonom_hardware'), 'config', 'hardware.yaml'),
            {
                'wheel_radius': ParameterValue(LaunchConfiguration('wheel_radius'), value_type=float),
                'wheel_separation': ParameterValue(LaunchConfiguration('wheel_separation'), value_type=float),
                'left_joint_name': LaunchConfiguration('left_wheel_joint'),
                'right_joint_name': LaunchConfiguration('right_wheel_joint'),
            },
            SIM_TIME,
        ],
        remappings=[('joint_states', '/joint_states'), ('wheel/odom', '/wheel/odom')],
        output='screen',
    )

    # Same IMU contract as the real robot: /imu/data with the bno055 driver's covariances (imu.yaml), not Gazebo's
    # noise stddev squared, which made the EKF trust acceleration spikes ~60x too much.
    imu_relay = Node(
        package='autonom_localization',
        executable='imu_covariance_relay',
        name='imu_covariance_relay',
        parameters=[os.path.join(bringup_share, 'config', 'imu.yaml'), SIM_TIME],
        remappings=[('imu_in', '/imu/raw'), ('imu_out', '/imu/data')],
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
        condition=IfCondition(LaunchConfiguration('use_autonomy')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bringup_share, 'config', 'autonom.rviz')],
        parameters=[SIM_TIME],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    world = GroupAction(scoped=False, condition=IfCondition(LaunchConfiguration('use_world')), actions=[
        robot_state_publisher,
        spawn_robot,
        bridge,
        wheel_odometry,
        imu_relay,
        rviz,
    ])

    return LaunchDescription(arguments + environment + [
        OpaqueFunction(function=_gazebo),
        world,
        autonomy,
    ])
