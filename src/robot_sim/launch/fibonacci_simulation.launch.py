"""Race arena simulation: tb3_simulation.launch.py on the Fibonacci map (models/Fibonacci_arena).

robot:=autonom   the real robot's model (autonom_description) with the base Nav2/patrol tuning (0.6 m/s)
robot:=tb3       the TurtleBot3 Burger placeholder with config/nav2_fibonacci.yaml (default)
robot:=tb3_fast  the TurtleBot3 body at the real robot's 0.6 m/s (urdf/turtlebot3_burger_fast.urdf.xacro,
                 config/nav2_fibonacci_fast.yaml)

The EKF fuses the IMU (gyro + accelerometer) and rf2o's forward speed by default (use_imu, use_rf2o): at
0.6 m/s the TurtleBot3's wheels slip and wheel-only odometry drifted up to 0.35 m / 1 rad; fused it stayed within 0.17 m over 3 minutes.

The robot spawns in the middle of the bottom corridor facing +x; laps run in the direction it faces at the
start, so set yaw:=3.1416 to race the other way round. Any tb3_simulation argument can be added.

competitors:=true (default, docs/07) adds two 12 cm x 12 cm competitor robots on the start line, competitor 1 to
the left and competitor 2 to the right of the main robot (which stays in the middle). They drive lanes 0.10 m left /
right of the track centre line (config/fibonacci_centre_line.yaml) at competitor_1_speed / competitor_2_speed,
in the main robot's race direction, and wait for /race/state RACING like the main robot (5 s start delay).
They stop while the main robot or the other competitor is right ahead (ground truth), so they never ram it.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOTS = ('tb3', 'tb3_fast', 'autonom')


def _simulation(context):
    sim_share = get_package_share_directory('robot_sim')
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ROBOTS:
        raise ValueError(f"robot must be one of {ROBOTS}, got '{robot}'")
    if robot == 'autonom':
        urdf = os.path.join(get_package_share_directory('autonom_description'), 'urdf', 'autonom.urdf.xacro')
        overlay = ''  # the base configuration is tuned for this robot
    elif robot == 'tb3_fast':
        urdf = os.path.join(sim_share, 'urdf', 'turtlebot3_burger_fast.urdf.xacro')
        overlay = os.path.join(sim_share, 'config', 'nav2_fibonacci_fast.yaml')
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
        'use_imu': LaunchConfiguration('use_imu'),
        'use_rf2o': LaunchConfiguration('use_rf2o'),
    }
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'tb3_simulation.launch.py')),
        launch_arguments=arguments.items(),
    )
    if LaunchConfiguration('competitors').perform(context).lower() != 'true':
        return [simulation]
    return [simulation] + _competitors(context, sim_share)


# (name, side: +1 left / -1 right of the main robot, speed argument)
COMPETITORS = (('competitor_1', 1.0, 'competitor_1_speed'), ('competitor_2', -1.0, 'competitor_2_speed'))
# Lateral spacing on the start line: competitor edges 0.14 m from the main robot's centre, outside the patrol's
# clearance (0.12 m for the TurtleBot3; closer, the start cell is not safe space and no goal is found), and 3 cm
# from the walls of the 0.58 m corridor.
START_SPACING = 0.20
LANE_OFFSET = 0.10


def _competitors(context, sim_share):
    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))
    # The centre line runs in the default race direction (spawned facing +x); facing the other way reverses it.
    reverse = math.cos(yaw) < 0.0
    with open(os.path.join(sim_share, 'models', 'competitor', 'competitor.sdf.template')) as f:
        template = f.read()
    centre_line = os.path.join(sim_share, 'config', 'fibonacci_centre_line.yaml')
    actions = []
    bridged = []
    for name, side, speed in COMPETITORS:
        # Left of the main robot = its heading turned +90 degrees.
        cx = x - side * START_SPACING * math.sin(yaw)
        cy = y + side * START_SPACING * math.cos(yaw)
        actions.append(Node(
            package='ros_gz_sim', executable='create', name=f'spawn_{name}', output='screen',
            arguments=['-world', 'default', '-name', name, '-string', template.replace('__NAME__', name),
                       '-x', f'{cx:.3f}', '-y', f'{cy:.3f}', '-z', '0.0', '-Y', f'{yaw:.4f}'],
        ))
        actions.append(Node(
            package='autonom_competitor', executable='competitor_driver', name=f'{name}_driver', output='screen',
            parameters=[centre_line, {
                'use_sim_time': True,
                'speed': float(LaunchConfiguration(speed).perform(context)),
                'lane_offset': side * LANE_OFFSET,
                'merge_distance': 1.5,
                'reverse': reverse,
                # Ground truth of the main robot and the other competitor: stop instead of ramming them.
                'others': ['/ground_truth/odom'] + [f'/{other}/odom' for other, _, _ in COMPETITORS if other != name],
            }],
            remappings=[('odom', f'/{name}/odom'), ('cmd_vel', f'/{name}/cmd_vel')],
        ))
        bridged += [f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry']
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='competitor_bridge', output='screen',
        arguments=bridged, parameters=[{'use_sim_time': True}],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tb3', description=f'Robot model: {" | ".join(ROBOTS)}'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-0.735'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='Fuse the simulated IMU (gyro yaw rate + forward acceleration) in the EKF'),
        DeclareLaunchArgument('use_rf2o', default_value='true', description='Fuse rf2o laser odometry in the EKF'),
        DeclareLaunchArgument('competitors', default_value='true',
                              description='Two simulated competitor robots beside the main robot (docs/07)'),
        DeclareLaunchArgument('competitor_1_speed', default_value='0.25', description='Left competitor, m/s'),
        DeclareLaunchArgument('competitor_2_speed', default_value='0.20', description='Right competitor, m/s'),
        OpaqueFunction(function=_simulation),
    ])
