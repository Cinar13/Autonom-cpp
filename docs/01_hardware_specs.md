# Hardware Specifications

## Compute & Deployment
*   **On-Robot Compute:** Raspberry Pi 5 (8GB). Runs sensor drivers, hardware interfaces, odometry fusion, SLAM, and Nav2.
*   **Off-Board / Base Station:** PC running Windows Subsystem for Linux (WSL). Strictly reserved for RViz2 visualization, remote monitoring, and debugging. No core mapping or control nodes run here.

## Sensors & Actuators
*   **LiDAR:** RPLiDAR S3 (driven via `sllidar_ros2`).
*   **Odometry Inputs:** 
    *   Wheel Encoders (Quadrature count via RPi 5 GPIO interrupts).
    *   Planar Laser Odometry (`rf2o_laser_odometry`).
    *   IMU (Optional. Software stack must dynamically enable/disable it).
*   **Motor Driver:** Dual A4950 H-bridge drivers. Driven via PWM and Direction GPIO pins from the RPi 5.
