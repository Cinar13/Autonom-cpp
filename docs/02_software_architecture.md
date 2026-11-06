# Software Architecture & ROS 2 Setup

## Core Stack
*   **ROS 2 Distribution:** Jazzy Jalisco.
*   **Language Standards:** C++17 strictly required for hardware control loops, drivers, and core nodes. Python is restricted to launch files, high-level lifecycle scripts, or Nav2 goal dispatching.

## Existing Workspace State
*   The `src/` directory contains `rf2o_laser_odometry` and `sllidar_ros2`. Focus generation on custom hardware integration packages, launch files, and parameters.

## Navigation & Odometry Pipeline
*   **Mapping:** `slam_toolbox` (async online mode) for real-time map building in unknown space.
*   **Sensor Fusion:** `robot_localization` (EKF).
    *   Primary Inputs: Encoder odometry (`/wheel/odom`) + `rf2o` odometry (`/rf2o/odom`).
    *   Conditional Input: IMU (`/imu/data`).
*   **Autonomy / Looping:** Configure Nav2 behavior trees or a Python action client node to send continuous loop waypoints through mapped regions once `slam_toolbox` publishes a valid map.
*   **Launch Architecture:** The main `bringup.launch.py` must include a `use_imu` boolean argument (default `false`). When `true`, launch the IMU node and supply IMU topics to the EKF node.

## Code Quality Standards
*   Follow ROS 2 C++ coding guidelines (snake_case topics/parameters, PascalCase node classes, camelCase functions).
*   Enforce RAII for hardware resources and handle zero-division in kinematics logic.
