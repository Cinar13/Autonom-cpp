# Autonom — ROS 2 Jazzy autonomous mapping & looping robot

Raspberry Pi 5 robot (A4950 motors, quadrature encoders, RPLiDAR S3, optional BNO055 IMU) that maps an
unknown environment with `slam_toolbox` and patrols continuous loops around the explored area with Nav2.
RViz runs on a WSL2 base station only.

| Package | Role |
|---|---|
| `autonom_hardware` | C++17 components: `MotorDriverNode` (cmd_vel → wheel PID → A4950 PWM), `WheelOdometryNode` (encoders → `/wheel/odom`, `/joint_states`), `OdometryCovarianceRelay` (adds covariance to rf2o) |
| `autonom_description` | URDF/xacro (`base_footprint`, `base_link`, `laser`, `imu_link`, wheel joints) |
| `autonom_bringup` | Launch files, EKF / slam_toolbox / Nav2 / CycloneDDS / RViz config, `loop_patrol.py` |
| `rf2o_laser_odometry`, `sllidar_ros2` | Upstream submodules |

```
/scan ─► rf2o ─► /rf2o/odom_raw ─► covariance relay ─► /rf2o/odom ─┐
encoders ─► wheel_odometry ─► /wheel/odom ─────────────────────────┼─► EKF ─► /odometry/filtered, TF odom→base_footprint
[use_imu] bno055 ─► /imu/data ──────────────────────────────────────┘
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
Run `wsl --shutdown`, then build only the visualisation packages (no GPIO library needed):
```bash
colcon build --symlink-install --packages-select autonom_description autonom_bringup
```

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

## Bring-up checklist

1. Verify pins, `pwm_chip` index (`ls /sys/class/pwm`) and `ticks_per_revolution` in `autonom_hardware/config/hardware.yaml`.
2. With wheels off the ground, check `/joint_states` velocities are positive when driving forward; fix with `*.inverted`.
3. Tune `pid.kf` first (effort per rad/s), then `pid.kp` / `pid.ki` live: `ros2 param set /motor_driver pid.kp 0.05`.
4. Check the LiDAR yaw (`laser_yaw` in the URDF) so the scan lines up with the robot's front in RViz.
