"""Nav2 servers for navigating inside a map that slam_toolbox is still building (no AMCL, map_server or docking)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from autonom_bringup.launch_utils import parameter_files

# Controller and behaviours feed the smoother; the smoother alone drives /cmd_vel.
NAV_TO_SMOOTHER = [('cmd_vel', 'cmd_vel_nav')]

NAV2_SERVERS = [
    ('nav2_controller', 'controller_server', NAV_TO_SMOOTHER),
    ('nav2_smoother', 'smoother_server', []),
    ('nav2_planner', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', NAV_TO_SMOOTHER),
    ('nav2_velocity_smoother', 'velocity_smoother', NAV_TO_SMOOTHER + [('cmd_vel_smoothed', 'cmd_vel')]),
    ('nav2_bt_navigator', 'bt_navigator', []),
    ('nav2_waypoint_follower', 'waypoint_follower', []),
]


def _nav2_nodes(context):
    parameters = parameter_files(context, LaunchConfiguration('params_file'), 'params_overlay')
    parameters.append({'use_sim_time': LaunchConfiguration('use_sim_time')})

    servers = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            parameters=parameters,
            remappings=remappings,
            respawn=True,
            respawn_delay=2.0,
            output='screen',
        )
        for package, executable, remappings in NAV2_SERVERS
    ]
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=parameters,
        output='screen',
    )
    return servers + [lifecycle_manager]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('autonom_bringup'), 'config', 'nav2.yaml']),
            description='Nav2 parameter file',
        ),
        DeclareLaunchArgument(
            'params_overlay', default_value='', description='Optional YAML layered over params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock (Gazebo)'),
        OpaqueFunction(function=_nav2_nodes),
    ])
