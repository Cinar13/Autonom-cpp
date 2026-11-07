"""Race arena simulation: tb3_simulation.launch.py on the Fibonacci map (models/Fibonacci_arena).

The robot spawns in the middle of the bottom corridor facing +x; laps run in the direction it faces at the
start, so set yaw:=3.1416 to race the other way round. Any tb3_simulation argument can be added.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('robot_sim')
    arguments = {
        'world': os.path.join(sim_share, 'worlds', 'fibonacci_harmonic.sdf'),
        'x': '0.0',
        'y': '-0.735',
        'yaw': '0.0',
        'nav2_params_overlay': os.path.join(sim_share, 'config', 'nav2_fibonacci.yaml'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'tb3_simulation.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in arguments}.items(),
        ),
    ])
