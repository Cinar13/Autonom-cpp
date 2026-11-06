# Autonom — ROS 2 Jazzy autonomous mapping & looping robot

Raspberry Pi 5 robot (A4950 motors, quadrature encoders, RPLiDAR S3, optional BNO055 IMU) that maps an
unknown environment with `slam_toolbox` and patrols continuous loops around the explored area with Nav2.
RViz runs on a WSL2 base station only. A Gazebo Harmonic simulation (TurtleBot3 Burger placeholder on the
`circuit2` track) runs the identical autonomy pipeline with `use_sim_time:=true`.

| Package | Role |
|---|---|
| `autonom_hardware` | C++17 components: `MotorDriverNode` (cmd_vel → wheel PID → A4950 PWM), `WheelOdometryNode` (encoders → `/wheel/odom`, `/joint_states`) |
| `autonom_localization` | C++17 components: `OdometryCovarianceRelay` (adds covariance to rf2o / Gazebo odometry), `ScanDecimator` (thins S3 scans for rf2o) |
| `autonom_description` | URDF/xacro (`base_footprint`, `base_link`, `laser`, `imu_link`, wheel joints); `gz_plugins.xacro` Gazebo macros incl. an RPLiDAR S3 sensor |
| `autonom_bringup` | Launch files (`bringup` = `hardware` + `autonomy`), EKF / slam_toolbox / Nav2 / CycloneDDS / RViz config, `loop_patrol.py` |
| `robot_sim` | Gazebo Harmonic: `circuit2_harmonic.sdf` world, `ros_gz_bridge` config, `tb3_simulation.launch.py`, sim Nav2 overlay |
| `turtlebot3_description` | Vendored Burger-only copy of ROBOTIS `turtlebot3_description` (Apache-2.0, see `VENDORED.md`) |
| `rf2o_laser_odometry`, `sllidar_ros2` | Upstream submodules |

```
[use_rf2o] /scan ─► decimator ─► rf2o ─► covariance relay ─► /rf2o/odom ─┐
encoders (or Gazebo /odom ─► relay) ─► /wheel/odom ─────────────────────┼─► EKF ─► /odometry/filtered, TF odom→base_footprint
[use_imu] bno055 (or Gazebo IMU) ─► /imu/data ───────────────────────────┘
/scan ─► slam_toolbox ─► /map, TF map→odom ─► Nav2 ◄─ loop_patrol (waypoints from explored free space)
Nav2 ─► /cmd_vel ─► motor_driver ◄─ /joint_states (wheel velocity feedback)
```

## Setup

```bash
git clone --recurse-submodules git@github.com:Cinar13/Autonom-cpp.git ~/Autonom-cpp
cd ~/Autonom-cpp
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
rosdep install --from-paths src --ignore-src -y --skip-keys cmake_modules   # rf2o lists an obsolete ROS 1 key
```

**Raspberry Pi 5** — enable hardware PWM in `/boot/firmware/config.txt`, then reboot:
```
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```
Give the user access to `/dev/gpiochip*`, `/sys/class/pwm` and the LiDAR serial port (group membership
or a udev rule, depending on the image; `dialout` covers the serial port), then build everything:
```bash
sudo usermod -aG dialout $USER
colcon build --symlink-install
```

**WSL2 base station** — use mirrored networking so the Pi can reach WSL directly. In `%UserProfile%\.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```
Run `wsl --shutdown`, then build everything (the simulation runs here too):
```bash
colcon build --symlink-install
```
WSL only: Mesa falls back to CPU rendering (`llvmpipe`), which makes Gazebo and RViz very slow. Routing
OpenGL to the GPU via `GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` restores speed but showed
rendering issues in Gazebo/RViz (untested further). Native Ubuntu with the NVIDIA driver needs neither.

## CycloneDDS (both machines, `~/.bashrc`)

```bash
source /opt/ros/jazzy/setup.bash
source ~/Autonom-cpp/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export AUTONOM_RPI_IP=192.168.1.50   # Raspberry Pi 5 LAN IP
export AUTONOM_WSL_IP=192.168.1.60   # Windows host LAN IP (= WSL IP in mirrored mode)
export CYCLONEDDS_URI=file://$HOME/Autonom-cpp/install/autonom_bringup/share/autonom_bringup/config/cyclonedds.xml
```
Allow UDP 7400–7700 inbound on the Windows firewall for the WSL network.

## Run

```bash
# Raspberry Pi 5
ros2 launch autonom_bringup bringup.launch.py                 # wheels + rf2o
ros2 launch autonom_bringup bringup.launch.py use_imu:=true   # + BNO055 fused in the EKF
ros2 launch autonom_bringup bringup.launch.py autonomous_loop:=false   # map/teleop only

# WSL2
ros2 launch autonom_bringup view.launch.py
```

Save the map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/autonom`.

## Simulation (WSL2, Gazebo Harmonic)

```bash
ros2 launch robot_sim tb3_simulation.launch.py                        # Gazebo GUI + RViz, TurtleBot3 Burger
ros2 launch robot_sim tb3_simulation.launch.py gui:=false use_rviz:=false   # headless
ros2 launch robot_sim tb3_simulation.launch.py use_rf2o:=true use_imu:=true # full sensor fusion
# Swap the placeholder for the custom robot (single argument):
ros2 launch robot_sim tb3_simulation.launch.py \
  robot_urdf:=$(ros2 pkg prefix autonom_description)/share/autonom_description/urdf/autonom.urdf.xacro
```

- `robot_urdf` is expanded with `sim:=true` and drives both `robot_state_publisher` and the Gazebo spawn; any
  URDF that uses the `gz_plugins.xacro` macros works with the same bridge (`robot_sim/config/bridge.yaml`).
- The simulated LiDAR matches the RPLiDAR S3 (3200 samples/rev at 10 Hz = 32 kHz, 0.1125°, 0.05–40 m), so
  `slam_toolbox.yaml` is shared unchanged; `nav2_sim.yaml` only overrides the Burger's footprint and speed.
- Bridged: `/clock`, `/scan`, `/cmd_vel`, `/odom` (relayed with covariance to `/wheel/odom`), `/joint_states`,
  `/imu/data`. Gazebo DiffDrive's TF is bridged to `/sim/tf`, not `/tf`: the EKF is the single owner of
  `odom→base_footprint`; `/tf_static` comes from `robot_state_publisher`.
- rf2o is off by default in simulation (`use_rf2o:=false`) to save CPU; it stays on for the real robot.
- `Circuit_ql_2` is vendored from gym-gazebo under GPL-3.0 (`robot_sim/models/Circuit_ql_2/VENDORED.md`).

## Bring-up checklist

1. Verify pins, `pwm_chip` index (`ls /sys/class/pwm`) and `ticks_per_revolution` in `autonom_hardware/config/hardware.yaml`.
2. With wheels off the ground, check `/joint_states` velocities are positive when driving forward; fix with `*.inverted`.
3. Tune `pid.kf` first (effort per rad/s), then `pid.kp` / `pid.ki` live: `ros2 param set /motor_driver pid.kp 0.05`.
4. Check the LiDAR yaw (`laser_yaw` in the URDF) so the scan lines up with the robot's front in RViz.
