# IMU gyro: specs and what the stack expects from it

Since `0c01346`, slam_toolbox trusts the odometry heading in scan matching (`angle_variance_penalty` 0.001,
`minimum_angle_penalty` 0.5; see `docs/HANDOFF.md`, "Map rotated at the first hairpin"). The EKF heading
comes from the gyro yaw rate plus the wheels, so on the real robot gyro bias now feeds the SLAM pose more directly.

**Which IMU:** `docs/01` lists the IMU as optional and doesn't name a model. The stack defaults to a
**Bosch BNO055** over I2C (`src/autonom_bringup/config/imu.yaml`, driver `ros-jazzy-bno055`). Confirm that this is
the part on the robot; if it isn't, swap it via `imu_package` / `imu_executable` / `imu_params_file`.

## BNO055 gyroscope (Bosch datasheet, typical values)

The figures below are typical values from the Bosch BNO055 datasheet. Check them against the revision you have.

| Property | Value |
|---|---|
| Range | ±125 / 250 / 500 / 1000 / 2000 °/s (selectable; fusion modes use 2000 °/s) |
| Resolution | 16 bit; 16 LSB per °/s at ±2000 °/s (the driver's `gyr_factor` 900 = LSB per rad/s) |
| Zero-rate offset (bias) | ±1 °/s typical, uncalibrated |
| Offset temperature drift | ±0.015 °/s per K |
| Noise density | 0.014 °/s/√Hz |
| Sensitivity tolerance | ±1 % (typ.); non-linearity 0.05 % of full scale |
| Fusion output rate | 100 Hz (the driver polls at `data_query_frequency` 50 Hz) |
| Interface | I2C, address 0x28 (bus 1); UART also possible |

What the bias means for a race: an uncalibrated 1 °/s (0.017 rad/s) bias integrates to about 0.28 rad over a
16 s lap. That is larger than the 0.12 rad SLAM yaw error measured in the sim. In NDOF mode (`operation_mode` 0x0C,
the configured mode) the BNO055 calibrates its own gyro offset; the residual after calibration is not measured yet.

## Configured values

| Where | Value | Note |
|---|---|---|
| `imu.yaml` `variance_angular_vel` | 0.0004 (rad/s)² → σ 0.02 rad/s | Covariance reported to the EKF; deliberately larger than the datasheet noise (≈0.1 °/s ≈ 0.002 rad/s over ~50 Hz) to leave room for bias |
| `ekf_imu.yaml` | gyro `vyaw` fused (`ax` not: on the real robot it made the pose run away), `imu0_angular_velocity_rejection_threshold` 5.0 | Absolute yaw (magnetometer) is not fused: unreliable near the motors |
| Sim (`gz_plugins.xacro` `gz_imu`) | white noise σ 2e-4 rad/s, **no bias**, 200 Hz (TB3) / 100 Hz (autonom) | Better than the real part: the sim doesn't test gyro bias or temperature drift |

## Before trusting it on the robot

1. Let the BNO055 reach gyro calibration status 3 (robot standing still for a few seconds after power-up; the driver
   publishes the calibration status at `calib_status_frequency` 0.1 Hz).
2. Measure the residual bias: robot still for 60 s, `ros2 topic echo /imu/data --field angular_velocity.z` and
   average it. Under about 0.1 °/s (0.002 rad/s) is fine for a 3-minute race with SLAM correcting.
3. Turn test: rotate the robot by exactly 360° on the spot (by hand or with `cmd_vel`). The `/odometry/filtered`
   yaw should change by 2π ± a few degrees.
4. If the bias is large, set the EKF's gyro covariance higher, or store calibration offsets in the driver
   (`set_offsets: true` with the measured offsets), rather than reverting the SLAM penalty.
