"""Real-robot hardware: robot description, motor/encoder drivers, RPLiDAR S3 and the optional IMU."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('autonom_bringup')
    hardware_params = PathJoinSubstitution([FindPackageShare('autonom_hardware'), 'config', 'hardware.yaml'])
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([FindPackageShare('autonom_description'), 'urdf', 'autonom.urdf.xacro'])]),
        value_type=str,
    )

    use_imu = LaunchConfiguration('use_imu')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    arguments = [
        DeclareLaunchArgument('use_imu', default_value='false', description='Start the IMU driver'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0', description='RPLiDAR S3 serial port'),
        DeclareLaunchArgument('imu_package', default_value='bno055', description='IMU driver package'),
        DeclareLaunchArgument('imu_executable', default_value='bno055', description='IMU driver executable'),
        DeclareLaunchArgument(
            'imu_params_file',
            default_value=PathJoinSubstitution([bringup_share, 'config', 'imu.yaml']),
            description='IMU driver parameter file',
        ),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Motor control and encoder feedback share one process so joint_states uses zero-copy intra-process transport.
    hardware_container = ComposableNodeContainer(
        name='hardware_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='autonom_hardware',
                plugin='autonom_hardware::WheelOdometryNode',
                name='wheel_odometry',
                parameters=[hardware_params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='autonom_hardware',
                plugin='autonom_hardware::MotorDriverNode',
                name='motor_driver',
                parameters=[hardware_params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_serial_port,
            'serial_baudrate': 1000000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'DenseBoost',
        }],
        output='screen',
    )

    imu = Node(
        package=LaunchConfiguration('imu_package'),
        executable=LaunchConfiguration('imu_executable'),
        parameters=[LaunchConfiguration('imu_params_file')],
        remappings=[('imu', '/imu/data')],
        condition=IfCondition(use_imu),
        output='screen',
    )

    return LaunchDescription(arguments + [robot_state_publisher, hardware_container, lidar, imu])
