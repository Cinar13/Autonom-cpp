# Hardware Specifications

## Compute & Deployment
*   **On-Robot Compute:** Raspberry Pi 5 (8GB). Runs sensor drivers, hardware interfaces, odometry fusion, SLAM, and Nav2.
*   **Off-Board / Base Station:** PC running native Ubuntu 24.04 (moved from WSL on 2026-11-07). Strictly reserved for RViz2 visualization, remote monitoring, and debugging. No core mapping or control nodes run here.

## Sensors & Actuators
*   **LiDAR:** RPLiDAR S3 (driven via `sllidar_ros2`).
*   **Odometry Inputs:** 
    *   Wheel Encoders (Quadrature count via RPi 5 GPIO interrupts).
    *   Planar Laser Odometry (`rf2o_laser_odometry`).
    *   IMU (Optional. Software stack must dynamically enable/disable it). The fitted board was sold as a BNO055 but is a
        **CEVA BNO08x** (SH-2 firmware 2.6.6) at **0x4B** on I2C1; it is read by `autonom_hardware`'s own `bno08x_node`
        (identified on the robot 2026-11-26).
*   **Motor Driver:** TB6612FNG dual H-bridge (VM 6 V from a buck regulator, logic at 3.3 V). Per motor one hardware PWM pin and two direction GPIOs (IN1/IN2) from the RPi 5, plus a shared STBY pin.

## Measured geometry (on the robot, 2026-11-26)
*   **Body:** 19 cm long, 17.5 cm wide. The wheel axle (`base_footprint`) is 6.5 cm from the back, so the body spans
    x = -0.065 .. +0.125 m and y = +-0.0875 m. Nav2 uses this rectangle as its footprint (plus 20 mm padding).
*   **Wheels:** 30-33 mm diameter (physical r = 0.016 m; effective rolling radius 0.0127 m, calibrated 2026-11-27), **14 cm apart** centre to centre (`wheel_separation: 0.14`).
*   **LiDAR:** centre 4.5 cm from the front, i.e. **8 cm ahead of the axle** (`laser_x: 0.08`), mounted turned 180 deg
    (`laser_yaw: pi`; checked on the robot: a forward push shows as +x in rf2o, and the corridor ahead appears ahead).
*   **Wiring vs. the original schematic:** both TB6612 channels and both encoder pairs are crossed. Left wheel = channel B
    (PWM GPIO13, BIN GPIO20/21) + encoder GPIO23/24; right wheel = channel A (PWM GPIO12, AIN GPIO5/6) + encoder GPIO17/27.
    `autonom_hardware/config/hardware.yaml` holds the real mapping.
*   **Power:** the Pi browned out / rebooted several times when the motors stalled at full duty (runaway test, wedged in a
    corner). The Pi and the motors should not share a weak supply.
