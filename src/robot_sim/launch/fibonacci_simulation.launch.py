"""Race arena simulation: tb3_simulation.launch.py on the Fibonacci map (models/Fibonacci_arena).

robot:=autonom   the real robot's model (autonom_description) with the base Nav2/patrol tuning (0.6 m/s)
robot:=tb3       the TurtleBot3 Burger placeholder with config/nav2_fibonacci.yaml (default)

The robot spawns in the middle of the bottom corridor facing +x; laps run in the direction it faces at the
start, so set yaw:=3.1416 to race the other way round. Any tb3_simulation argument can be added.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROBOTS = ('tb3', 'autonom')


def _simulation(context):
    sim_share = get_package_share_directory('robot_sim')
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ROBOTS:
        raise ValueError(f"robot must be one of {ROBOTS}, got '{robot}'")
    if robot == 'autonom':
        urdf = os.path.join(get_package_share_directory('autonom_description'), 'urdf', 'autonom.urdf.xacro')
        overlay = ''  # the base configuration is tuned for this robot
    else:
        urdf = os.path.join(sim_share, 'urdf', 'turtlebot3_burger_sim.urdf.xacro')
        overlay = os.path.join(sim_share, 'config', 'nav2_fibonacci.yaml')
    arguments = {
        'world': os.path.join(sim_share, 'worlds', 'fibonacci_harmonic.sdf'),
        'robot_urdf': urdf,
        'nav2_params_overlay': overlay,
        'x': LaunchConfiguration('x'),
        'y': LaunchConfiguration('y'),
        'yaw': LaunchConfiguration('yaw'),
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'tb3_simulation.launch.py')),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tb3', description=f'Robot model: {" | ".join(ROBOTS)}'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-0.735'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_simulation),
    ])
