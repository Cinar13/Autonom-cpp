# Gazebo Simulation Specifications

## Simulation Environment
*   **Simulator:** Gazebo Harmonic (`ros_gz`).
*   **World File:** `circuit2_harmonic.sdf`.
*   **Time Synchronization:** All simulation nodes and software stack components must launch with `use_sim_time:=true`.

## Robot Platform (Temporary Placeholder)
*   **Model:** TurtleBot3 Burger (`TURTLEBOT3_MODEL=burger`).
*   **URDF Integration:** Use standard TurtleBot3 Burger URDF/Xacro for all TF tree generation, sensor placements, and wheel kinematics until the custom robot URDF file is finalized.
*   **Modularity:** Structure the launch architecture so replacing the TurtleBot3 URDF with the custom robot URDF requires changing only a single launch argument or configuration parameter.

## ROS 2 Gazebo Bridge (`ros_gz_bridge`)
Configure ROS-Gazebo bridge topic remaps for:
*   `/scan` (LiDAR scan output from RPLiDAR model / simulated ray sensor)
*   `/cmd_vel` (Velocity commands from Nav2 controller)
*   `/odom` (Simulated wheel encoder odometry)
*   `/clock` (Gazebo clock synchronization)
*   `/tf` and `/tf_static` (Frame transforms)

## Pipeline Verification
*   Feed the simulated `/scan` topic into `rf2o_laser_odometry`.
*   Fuse simulated wheel odometry and `rf2o` scan matching in `robot_localization` EKF.
*   Execute `slam_toolbox` (async online mode) and Nav2 autonomous looping on the Gazebo simulation instance.
