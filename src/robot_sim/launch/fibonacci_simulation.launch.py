"""Race arena simulation: tb3_simulation.launch.py on the Fibonacci map (models/Fibonacci_arena).

robot:=autonom   the real robot's model (autonom_description) with the base Nav2/patrol tuning (0.6 m/s)
robot:=tb3       the TurtleBot3 Burger placeholder with config/nav2_fibonacci.yaml (default)
robot:=tb3_fast  the TurtleBot3 body at the real robot's 0.6 m/s (urdf/turtlebot3_burger_fast.urdf.xacro,
                 config/nav2_fibonacci_fast.yaml)

mapping_speed:=0.3 race_speed:=0.6 set the patrol's cruise speeds (m/s) for the mapping lap and the race laps;
empty keeps the configured values. Limits: 0.1 up to the robot's tuned maximum (tb3 0.22, tb3_fast and autonom 0.6).

The EKF fuses the IMU (gyro + accelerometer) and rf2o's forward speed by default (use_imu, use_rf2o): at
0.6 m/s the TurtleBot3's wheels slip and wheel-only odometry drifted up to 0.35 m / 1 rad; fused it stayed within 0.17 m over 3 minutes.

The robot spawns in the middle of the bottom corridor facing +x; laps run in the direction it faces at the
start, so set yaw:=3.1416 to race the other way round. Any tb3_simulation argument can be added.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROBOTS = ('tb3', 'tb3_fast', 'autonom')
# Highest cruise speed each robot is tuned for (Nav2 velocity_smoother / Gazebo limits), m/s.
MAX_SPEED = {'tb3': 0.22, 'tb3_fast': 0.6, 'autonom': 0.6}
MIN_SPEED = 0.1


def _speed_overlay(context, robot):
    """Parameter overlay for loop_patrol's exploration_speed / race_speed from the launch arguments, or ''."""
    speeds = {}
    for argument, parameter in (('mapping_speed', 'exploration_speed'), ('race_speed', 'race_speed')):
        text = LaunchConfiguration(argument).perform(context).strip()
        if not text:
            continue
        try:
            value = float(text)
        except ValueError:
            raise ValueError(f"{argument} must be a number in m/s, got '{text}'") from None
        if not MIN_SPEED <= value <= MAX_SPEED[robot]:
            raise ValueError(f'{argument} {value} m/s is outside {MIN_SPEED}..{MAX_SPEED[robot]} m/s for robot:={robot}')
        speeds[parameter] = value
    if not speeds:
        return ''
    with tempfile.NamedTemporaryFile('w', prefix='autonom_speed_', suffix='.yaml', delete=False) as f:
        f.write('loop_patrol:\n  ros__parameters:\n')
        for parameter, value in speeds.items():
            f.write(f'    {parameter}: {value}\n')
    print(f"[fibonacci_simulation] speeds: {', '.join(f'{k} {v} m/s' for k, v in speeds.items())}")
    return f.name


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
    speed_overlay = _speed_overlay(context, robot)
    if speed_overlay:
        overlay = ','.join(path for path in (overlay, speed_overlay) if path)
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
        DeclareLaunchArgument('mapping_speed', default_value='',
                              description='Mapping-lap cruise speed, m/s (empty: loop_patrol.yaml / robot overlay)'),
        DeclareLaunchArgument('race_speed', default_value='',
                              description='Race-lap cruise speed, m/s (empty: loop_patrol.yaml / robot overlay)'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='Fuse the simulated IMU (gyro yaw rate + forward acceleration) in the EKF'),
        DeclareLaunchArgument('use_rf2o', default_value='true', description='Fuse rf2o laser odometry in the EKF'),
        OpaqueFunction(function=_simulation),
    ])
