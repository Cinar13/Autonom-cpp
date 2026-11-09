"""Race arena simulation: tb3_simulation.launch.py on the Fibonacci map (models/Fibonacci_arena).

robot:=autonom   the real robot's model (autonom_description) with the base Nav2/patrol tuning (0.6 m/s)
robot:=tb3       the TurtleBot3 Burger placeholder with config/nav2_fibonacci.yaml (default)
robot:=tb3_fast  the TurtleBot3 body at the real robot's 0.6 m/s (urdf/turtlebot3_burger_fast.urdf.xacro,
                 config/nav2_fibonacci_fast.yaml)

Driving overrides (empty keeps the configured values):
  mapping_speed:= race_speed:=   cruise speed of the mapping lap / race laps, 0.1-1.5 m/s (autonom: up to 1.46, its
                                 motors' limit; the stack is tuned for 0.3 / 0.6)
  mapping_accel:= race_accel:=   linear acceleration limit, 0.1-3.0 m/s^2 (configured 0.5 / 1.5)
  turn_speed:=                   turn speed limit, 0.5-5.0 rad/s (configured 3.0; tb3 2.0)
  controller:=mppi               RPP maps, MPPI races (autonom_bringup/config/nav2_mppi.yaml + config/nav2_mppi.yaml)
  controller:=graceful           Nav2's Graceful controller (config/nav2_graceful.yaml)
  planner:=smac                  Smac 2D global planner instead of NavFn (autonom_bringup/config/nav2_smac.yaml)
  planner:=thetastar             Theta* any-angle global planner (autonom_bringup/config/nav2_thetastar.yaml)
The Nav2 velocity smoother and the Gazebo DiffDrive limits are raised to match.

The EKF fuses the IMU (gyro + accelerometer) and rf2o's forward speed by default (use_imu, use_rf2o): at
0.6 m/s the TurtleBot3's wheels slip and wheel-only odometry drifted up to 0.35 m / 1 rad; fused it stayed within 0.17 m over 3 minutes.

The robot spawns in the middle of the bottom corridor facing +x; laps run in the direction it faces at the
start, so set yaw:=3.1416 to race the other way round. Any tb3_simulation argument can be added.
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROBOTS = ('tb3', 'tb3_fast', 'autonom')
# Per robot: the configured Nav2 velocity_smoother limits, the Gazebo DiffDrive limits of its model, and the highest
# cruise speed a user may ask for (the real robot's motors stop at 430 rpm = 1.46 m/s).
LIMITS = {
    'tb3': {'nav_speed': 0.22, 'nav_turn': 2.0, 'gz_speed': 0.22, 'gz_turn': 2.84, 'max_speed': 1.5},
    'tb3_fast': {'nav_speed': 0.6, 'nav_turn': 3.0, 'gz_speed': 0.65, 'gz_turn': 4.0, 'max_speed': 1.5},
    'autonom': {'nav_speed': 0.6, 'nav_turn': 3.0, 'gz_speed': 1.46, 'gz_turn': 3.0, 'max_speed': 1.46},
}
MIN_SPEED = 0.1
ACCEL_RANGE = (0.1, 3.0)  # m/s^2
TURN_RANGE = (0.5, 5.0)   # rad/s
GZ_ACCEL = 1.0            # Gazebo DiffDrive max_linear_acceleration of every model
# Angular parts of loop_patrol.yaml exploration_max_accel / race_max_accel (rad/s^2).
TURN_ACCEL = {'exploration_max_accel': 2.0, 'race_max_accel': 5.0}


def _number(context, argument, low, high, unit):
    text = LaunchConfiguration(argument).perform(context).strip()
    if not text:
        return None
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"{argument} must be a number in {unit}, got '{text}'") from None
    if not low <= value <= high:
        raise ValueError(f'{argument} {value} {unit} is outside {low}..{high} {unit}')
    return value


def _driving_overrides(context, robot):
    """Parameter overlay and xacro arguments for user-chosen speeds, accelerations and turn speed ('' when none).

    Speeds and accelerations go to loop_patrol, which switches them live between the mapping lap and the race;
    the Nav2 velocity_smoother, RPP's turn-in-place rate and the Gazebo DiffDrive limits are raised to match."""
    limits = LIMITS[robot]
    mapping_speed = _number(context, 'mapping_speed', MIN_SPEED, limits['max_speed'], 'm/s')
    race_speed = _number(context, 'race_speed', MIN_SPEED, limits['max_speed'], 'm/s')
    mapping_accel = _number(context, 'mapping_accel', *ACCEL_RANGE, 'm/s^2')
    race_accel = _number(context, 'race_accel', *ACCEL_RANGE, 'm/s^2')
    turn_speed = _number(context, 'turn_speed', *TURN_RANGE, 'rad/s')
    patrol, smoother, controller, xacro = {}, {}, {}, {}
    if mapping_speed is not None:
        patrol['exploration_speed'] = mapping_speed
    if race_speed is not None:
        patrol['race_speed'] = race_speed
    if mapping_accel is not None:
        patrol['exploration_max_accel'] = [mapping_accel, 0.0, TURN_ACCEL['exploration_max_accel']]
    if race_accel is not None:
        patrol['race_max_accel'] = [race_accel, 0.0, TURN_ACCEL['race_max_accel']]
        smoother['max_accel'] = [race_accel, 0.0, TURN_ACCEL['race_max_accel']]
    top = max([v for v in (mapping_speed, race_speed) if v is not None] + [limits['nav_speed']])
    turn = turn_speed if turn_speed is not None else limits['nav_turn']
    if top > limits['nav_speed'] or turn_speed is not None:
        smoother['max_velocity'] = [top, 0.0, turn]
        smoother['min_velocity'] = [-0.15, 0.0, -turn]
    if turn_speed is not None:
        controller['FollowPath'] = {'rotate_to_heading_angular_vel': turn_speed}
        xacro['gz_max_angular_velocity'] = max(limits['gz_turn'], turn_speed)
    if top + 0.05 > limits['gz_speed'] and robot != 'autonom':
        xacro['gz_max_velocity'] = round(top + 0.05, 3)
    accels = [a for a in (mapping_accel, race_accel) if a is not None]
    if accels and max(accels) > GZ_ACCEL:
        xacro['gz_max_acceleration'] = max(accels)
    if not (patrol or smoother or controller):
        return '', ''
    sections = {'loop_patrol': patrol, 'velocity_smoother': smoother, 'controller_server': controller}
    with tempfile.NamedTemporaryFile('w', prefix='autonom_driving_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump({node: {'ros__parameters': p} for node, p in sections.items() if p}, f)
    chosen = {k: v for k, v in (('mapping_speed', mapping_speed), ('race_speed', race_speed),
                                ('mapping_accel', mapping_accel), ('race_accel', race_accel),
                                ('turn_speed', turn_speed)) if v is not None}
    print(f'[fibonacci_simulation] driving: {chosen}' + ('; above 0.6 m/s Nav2 is untuned' if top > 0.6 else ''))
    return f.name, ' '.join(f'{k}:={v}' for k, v in xacro.items())


def _simulation(context):
    sim_share = get_package_share_directory('robot_sim')
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ROBOTS:
        raise ValueError(f"robot must be one of {ROBOTS}, got '{robot}'")
    wheels = {}  # tb3_simulation defaults: the TurtleBot3's wheels
    if robot == 'autonom':
        urdf = os.path.join(get_package_share_directory('autonom_description'), 'urdf', 'autonom.urdf.xacro')
        overlay = ''  # the base configuration is tuned for this robot
        # Keep in sync with autonom.urdf.xacro and autonom_hardware hardware.yaml.
        wheels = {'wheel_radius': '0.0325', 'wheel_separation': '0.17',
                  'left_wheel_joint': 'left_wheel_joint', 'right_wheel_joint': 'right_wheel_joint'}
    elif robot == 'tb3_fast':
        urdf = os.path.join(sim_share, 'urdf', 'turtlebot3_burger_fast.urdf.xacro')
        overlay = os.path.join(sim_share, 'config', 'nav2_fibonacci_fast.yaml')
    else:
        urdf = os.path.join(sim_share, 'urdf', 'turtlebot3_burger_sim.urdf.xacro')
        overlay = os.path.join(sim_share, 'config', 'nav2_fibonacci.yaml')
    controller = LaunchConfiguration('controller').perform(context)
    if controller not in ('rpp', 'mppi', 'graceful'):
        raise ValueError(f"controller must be rpp, mppi or graceful, got '{controller}'")
    bringup_config = os.path.join(get_package_share_directory('autonom_bringup'), 'config')
    if controller == 'mppi':  # the robot's MPPI overlay, then the TurtleBot3 delta
        overlay = ','.join(p for p in (overlay, os.path.join(bringup_config, 'nav2_mppi.yaml'),
                                       os.path.join(sim_share, 'config', 'nav2_mppi.yaml')) if p)
    elif controller == 'graceful':  # sim only: much slower than RPP and MPPI (docs/SESSION_2026-11-09.md)
        overlay = ','.join(p for p in (overlay, os.path.join(sim_share, 'config', 'nav2_graceful.yaml')) if p)
    planner = LaunchConfiguration('planner').perform(context)
    if planner not in ('navfn', 'smac', 'thetastar'):
        raise ValueError(f"planner must be navfn, smac or thetastar, got '{planner}'")
    if planner != 'navfn':  # shared with the robot (bringup.launch.py planner:=)
        overlay = ','.join(p for p in (overlay, os.path.join(bringup_config, f'nav2_{planner}.yaml')) if p)
    driving_overlay, xacro_args = _driving_overrides(context, robot)
    if driving_overlay:
        overlay = ','.join(path for path in (overlay, driving_overlay) if path)
    arguments = {
        'world': os.path.join(sim_share, 'worlds', 'fibonacci_harmonic.sdf'),
        'robot_urdf': urdf,
        'nav2_params_overlay': overlay,
        'robot_xacro_args': xacro_args,
        'x': LaunchConfiguration('x'),
        'y': LaunchConfiguration('y'),
        'yaw': LaunchConfiguration('yaw'),
        'use_imu': LaunchConfiguration('use_imu'),
        'use_rf2o': LaunchConfiguration('use_rf2o'),
        **wheels,
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'tb3_simulation.launch.py')),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tb3', description=f'Robot model: {" | ".join(ROBOTS)}'),
        DeclareLaunchArgument(
            'controller', default_value='rpp',
            description='Path follower: rpp (Regulated Pure Pursuit) | mppi (Model Predictive Path Integral) | graceful'),
        DeclareLaunchArgument(
            'planner', default_value='navfn',
            description='Global planner: navfn (shortest grid path) | smac (Smac 2D, smoothed grid path) | thetastar (any-angle)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-0.735'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('mapping_speed', default_value='', description='Mapping-lap cruise speed, m/s'),
        DeclareLaunchArgument('race_speed', default_value='', description='Race-lap cruise speed, m/s'),
        DeclareLaunchArgument('mapping_accel', default_value='', description='Mapping-lap acceleration, m/s^2'),
        DeclareLaunchArgument('race_accel', default_value='', description='Race-lap acceleration, m/s^2'),
        DeclareLaunchArgument('turn_speed', default_value='', description='Turn speed limit, rad/s'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='Fuse the simulated IMU (gyro yaw rate + forward acceleration) in the EKF'),
        DeclareLaunchArgument('use_rf2o', default_value='true', description='Fuse rf2o laser odometry in the EKF'),
        OpaqueFunction(function=_simulation),
    ])
