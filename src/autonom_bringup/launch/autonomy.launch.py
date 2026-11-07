"""Robot-agnostic autonomy pipeline shared by the real robot and Gazebo.

/wheel/odom -> EKF (+ optional rf2o laser odometry and /imu/data) -> slam_toolbox -> Nav2 -> loop patrol.
Nav2 -> /cmd_vel_smoothed -> race start gate -> /cmd_vel: nothing moves until 5 s after the start signal (AC-3.2).
Expects /scan, /wheel/odom, /tf_static and (when use_imu) /imu/data from the robot or simulator.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

from autonom_bringup.launch_utils import parameter_files


def _pipeline(context):
    config_dir = PathJoinSubstitution([FindPackageShare('autonom_bringup'), 'config'])
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = {'use_sim_time': use_sim_time}
    slam_params = parameter_files(
        context, PathJoinSubstitution([config_dir, 'slam_toolbox.yaml']), 'slam_params_overlay')
    nav2_overlay = LaunchConfiguration('nav2_params_overlay')
    patrol_params = parameter_files(
        context, PathJoinSubstitution([config_dir, 'loop_patrol.yaml']), 'nav2_params_overlay')

    rf2o_enabled = IfCondition(LaunchConfiguration('use_rf2o'))
    scan_decimator = Node(
        package='autonom_localization',
        executable='scan_decimator',
        name='rf2o_scan_decimator',
        parameters=[{'decimation': LaunchConfiguration('rf2o_scan_decimation')}, sim_time],
        remappings=[('scan_in', '/scan'), ('scan_out', '/rf2o/scan')],
        condition=rf2o_enabled,
        output='screen',
    )
    laser_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic': '/rf2o/scan',
            'odom_topic': '/rf2o/odom_raw',
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
        }, sim_time],
        condition=rf2o_enabled,
        output='screen',
    )
    rf2o_relay = Node(
        package='autonom_localization',
        executable='odometry_covariance_relay',
        name='rf2o_covariance_relay',
        parameters=[PathJoinSubstitution([config_dir, 'odometry_relays.yaml']), sim_time],
        remappings=[('odom_in', '/rf2o/odom_raw'), ('odom_out', '/rf2o/odom')],
        condition=rf2o_enabled,
        output='screen',
    )

    speckle_filter = Node(
        package='autonom_localization',
        executable='scan_speckle_filter',
        name='scan_speckle_filter',
        parameters=[sim_time],
        remappings=[('scan_in', '/scan'), ('scan_out', '/scan_filtered')],
        output='screen',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('autonom_bringup'), 'launch', 'localization.launch.py'])),
        launch_arguments={
            'use_rf2o': LaunchConfiguration('use_rf2o'),
            'use_imu': LaunchConfiguration('use_imu'),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Phase 1 maps asynchronously; with localize_after_mapping the map_and_localization node (same mapping code
    # path as the async node) is switched to pure scan-matching localization by the loop patrol once the lap is
    # driven (loop_patrol.localize_after_laps), so the map stops changing.
    localize_after_mapping = IfCondition(LaunchConfiguration('localize_after_mapping')).evaluate(context)
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='map_and_localization_slam_toolbox_node' if localize_after_mapping else 'async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=slam_params + [{'use_lifecycle_manager': False}, sim_time],
        output='screen',
    )
    slam_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_CONFIGURE))
    slam_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam,
        start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_ACTIVATE))],
    ))

    navigation = TimerAction(
        period=LaunchConfiguration('nav2_startup_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('autonom_bringup'), 'launch', 'navigation.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_overlay': nav2_overlay,
                'cmd_vel_out_topic': '/cmd_vel_smoothed',
            }.items(),
        )],
    )

    start_gate = Node(
        package='autonom_race',
        executable='race_start_gate',
        name='race_start_gate',
        parameters=[{
            'start_delay': LaunchConfiguration('start_delay'),
            'auto_start': LaunchConfiguration('auto_start'),
        }, sim_time],
        remappings=[('cmd_vel_in', '/cmd_vel_smoothed'), ('cmd_vel_out', '/cmd_vel')],
        output='screen',
    )

    loop_patrol = Node(
        package='autonom_patrol',
        executable='loop_patrol',
        name='loop_patrol',
        parameters=patrol_params + [sim_time] + ([] if localize_after_mapping else [{'localize_after_laps': 0}]),
        condition=IfCondition(LaunchConfiguration('autonomous_loop')),
        output='screen',
    )

    return [speckle_filter, scan_decimator, laser_odometry, rf2o_relay, localization, slam_activate, slam, slam_configure, start_gate, navigation, loop_patrol]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock (Gazebo)'),
        DeclareLaunchArgument(
            'use_rf2o', default_value='true', description='Run rf2o laser odometry and fuse it in the EKF'),
        DeclareLaunchArgument('use_imu', default_value='false', description='Fuse /imu/data in the EKF'),
        DeclareLaunchArgument(
            'localize_after_mapping', default_value='true',
            description='Freeze the map once the lap is driven and localize by scan matching (false: map all race)'),
        DeclareLaunchArgument(
            'autonomous_loop', default_value='true', description='Run the loop patrol once Nav2 and the map are up'),
        DeclareLaunchArgument(
            'start_delay', default_value='5.0',
            description='Seconds between the start signal and the first motion (competition rule AC-3.2: 5.0)'),
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description='Start the race on launch instead of waiting for /race/start_signal or /race/start'),
        DeclareLaunchArgument(
            'nav2_startup_delay', default_value='5.0',
            description='Seconds to wait for TF (map->odom->base) before starting Nav2'),
        DeclareLaunchArgument(
            'rf2o_scan_decimation', default_value='4',
            description='Feed rf2o every Nth beam (3200-beam S3 scans saturate its single thread)'),
        DeclareLaunchArgument(
            'slam_params_overlay', default_value='', description='Optional YAML layered over slam_toolbox.yaml'),
        DeclareLaunchArgument(
            'nav2_params_overlay', default_value='',
            description='Optional YAML layered over nav2.yaml and loop_patrol.yaml'),
        OpaqueFunction(function=_pipeline),
    ])
