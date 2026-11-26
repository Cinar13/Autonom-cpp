# Autonom — ROS 2 Jazzy autonomous mapping & racing robot

Raspberry Pi 5 robot (TB6612FNG motor driver, N20 300 rpm 1:50 motors with 7 PPR quadrature encoders, RPLiDAR S3, optional BNO055 IMU) for the
Fibonacci Robot Olympiad race: it maps the unknown track with `slam_toolbox` on its first lap, freezes the map and
then races continuous laps with Nav2. A Gazebo Harmonic simulation of the race arena runs the identical autonomy
pipeline with `use_sim_time:=true`; the base station (Ubuntu 24.04) runs the simulation and RViz.

**Current state, open issues and the test plan: [`docs/HANDOFF.md`](docs/HANDOFF.md).** Latest results:
[`docs/REPORT.md`](docs/REPORT.md).

| Package | Role |
|---|---|
| `autonom_hardware` | C++17 components: `MotorDriverNode` (cmd_vel → wheel PID → TB6612FNG PWM), `WheelOdometryNode` (encoders → `/wheel/odom`, `/joint_states`) |
| `autonom_localization` | C++17 components: odometry and IMU covariance relays, `ScanDecimator` (thins S3 scans for rf2o), `ScanSpeckleFilter` (drops isolated returns before the costmaps) |
| `autonom_patrol` | C++17 loop patrol: track planner (exploration, laps by driving distance, rear gate, start line) and the Nav2 patrol node |
| `autonom_race` | Competition rules in software: race start gate (5 s start delay, blocks `/cmd_vel` until then, race state) |
| `autonom_description` | URDF/xacro (`base_footprint`, `base_link`, `laser`, `imu_link`, wheel joints); `gz_plugins.xacro` Gazebo macros incl. an RPLiDAR S3 sensor |
| `autonom_bringup` | Launch files (`bringup` = `hardware` + `autonomy`), EKF / slam_toolbox / Nav2 / patrol / CycloneDDS / RViz config, race behavior trees, Nav2 mode overlays |
| `robot_sim` | Gazebo Harmonic: Fibonacci race arena (and the older `circuit2` track), `ros_gz_bridge` config, TurtleBot3 overlays, sim launch |
| `turtlebot3_description` | Vendored Burger-only copy of ROBOTIS `turtlebot3_description` (Apache-2.0, see `VENDORED.md`) |
| `rf2o_laser_odometry`, `sllidar_ros2` | Upstream submodules (unmodified) |

```
[use_rf2o] /scan ─► decimator ─► rf2o ─► covariance relay ─► /rf2o/odom ─┐
encoders (sim: Gazebo wheel joints) ─► /wheel/odom ──────────────────────┼─► EKF ─► /odometry/filtered, TF odom→base_footprint
[use_imu] bno055 (sim: Gazebo IMU ─► covariance relay) ─► /imu/data ─────┘
/scan ─► slam_toolbox ─► /map, TF map→odom
/scan ─► speckle filter ─► /scan_filtered ─► Nav2 costmaps
loop_patrol ─► Nav2 (exploration goals, then one NavigateThroughPoses goal per lap)
Nav2 ─► /cmd_vel_smoothed ─► race_start_gate ─► /cmd_vel ─► motor_driver (sim: Gazebo DiffDrive)
```

The EKF is the only owner of `odom→base_footprint`; only `race_start_gate` publishes `/cmd_vel`.

## Setup

One-shot on Ubuntu 24.04 (ROS 2 Jazzy, Gazebo Harmonic bridge, clone, rosdep, build):

```bash
bash scripts/install.sh --branch optimised
```

Or by hand:

```bash
git clone --recurse-submodules git@github.com:Cinar13/Autonom-cpp.git ~/Autonom-cpp
cd ~/Autonom-cpp
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-rmw-cyclonedds-cpp python3-rosdep
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --skip-keys cmake_modules   # rf2o lists an obsolete ROS 1 key
colcon build --symlink-install
colcon test && colcon test-result --all
```

**Raspberry Pi 5** — enable hardware PWM in `/boot/firmware/config.txt`, then reboot:
```
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```
### Pinout (Raspberry Pi 5)

BCM GPIO numbers, as used in `autonom_hardware/config/hardware.yaml`, following the wiring schematic: a
TB6612FNG drives both motors (VM from the 6 V buck regulator, VCC from the Pi's 3.3 V), channel B the left
motor and channel A the right. Both the motor channels and the encoder pairs are wired crossed with respect to
the original schematic (verified on the robot 2026-11-26); the table shows the real wiring.

| Function | BCM GPIO | Header pin | Parameter |
|---|---|---|---|
| Left motor PWMB (PWM0 ch 1) | 13 | 33 | `left_motor.pwm_channel: 1` |
| Left motor BIN1 / BIN2 | 20 / 21 | 38 / 40 | `left_motor.in1_pin` / `in2_pin` |
| Right motor PWMA (PWM0 ch 0) | 12 | 32 | `right_motor.pwm_channel: 0` |
| Right motor AIN1 / AIN2 | 5 / 6 | 29 / 31 | `right_motor.in1_pin` / `in2_pin` |
| TB6612 STBY (high = enabled) | 26 | 37 | `standby_pin` |
| Start button (other side to GND) | 16 | 36 | `start_button.pin` |
| Left encoder A / B | 23 / 24 | 16 / 18 | `left_encoder.pin_a` / `pin_b` |
| Right encoder A / B | 17 / 27 | 11 / 13 | `right_encoder.pin_a` / `pin_b` |
| BNO055 IMU SDA / SCL (I2C1, address 0x28) | 2 / 3 | 3 / 5 | `autonom_bringup/config/imu.yaml` |
| RPLiDAR S3 | USB (`/dev/ttyUSB0`, 1 000 000 baud) | — | `lidar_serial_port` launch argument |

Encoders and the IMU run from 3.3 V: the Pi's GPIO pins are not 5 V tolerant, so never pull encoder outputs
up to 5 V (the internal pull-ups are enabled with `pull_up: true`). The start button uses the internal pull-up
too (pressed = low, 30 ms debounce) and publishes `/race/start_signal`; it must be seen released once after
start-up before a press counts, so a stuck or shorted button can never start the race.

### Checking encoders and motors (any new build, wheels off the ground)

Sign convention: a positive wheel velocity means that wheel rolls the robot forward. Each side meets it on its
own through two independent flags, `<side>_motor.inverted` and `<side>_encoder.inverted`. With the current
wiring only `left_encoder.inverted` is `true`; both motors are `false` (IN1 high = forward on both sides). The
sign belongs to the encoder and to the motor lead wiring, not to the side: if pairs are ever re-wired, re-check
them. Get each flag right on its own, or the speed
loop runs away.

1. **Encoders, motors unpowered.** Standing behind the robot, turn one wheel one turn forward by hand. That
   wheel's joint in `/joint_states` must change by about +2π (+6.28 rad) and the other joint must not change.
   The wrong joint moves: the pin pairs are swapped. It moves by −2π: flip that encoder's `inverted`.
2. **One motor at a time, closed loop.** Cancel one wheel with the turn rate: `v = 0.05`, `w = ±v / (b/2) = ±0.588`
   (b = 0.17 m) gives one wheel 2v/r = 3.08 rad/s and the other 0. Expect a steady speed near the target. If it
   races to full speed, the motor and encoder signs disagree; with the encoder already passing step 1, flip that
   motor's `inverted`. Stop as soon as a wheel goes above ~10 rad/s: a runaway at full duty browned out the Pi twice.
3. **Only one driver process.** Two leftover `motor_driver` instances fighting over the same pins give
   back-and-forth motion that looks like a wiring fault (`pgrep -a motor_driver`).

Scaling: 7 PPR × 4 (quadrature) × 50 (gearbox) = 1400 counts per wheel revolution. Speed is Δcounts / dt at
50 Hz, so at 3 rad/s that is ~13 counts per cycle and one count of jitter is ±0.22 rad/s (the 2.9 / 3.1 rad/s
flicker is normal). If the PID needs a smoother signal at low speed, use a longer window or a light low-pass
(α ≈ 0.3); don't add derivative gain on the raw signal.

Give the user access to `/dev/gpiochip*`, `/sys/class/pwm` and the LiDAR serial port (group membership
or a udev rule, depending on the image; `dialout` covers the serial port), then build everything:
```bash
sudo usermod -aG dialout $USER
colcon build --symlink-install
```
If CMake reports a non-existent libgpiod include path, the build cache points at an old install; rebuild
with `colcon build --symlink-install --packages-select autonom_hardware --cmake-clean-cache`.

## CycloneDDS (both machines, `~/.bashrc`)

```bash
source /opt/ros/jazzy/setup.bash
source ~/Autonom-cpp/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export AUTONOM_RPI_IP=192.168.1.50   # Raspberry Pi 5 LAN IP
export AUTONOM_WSL_IP=192.168.1.60   # base station LAN IP (the name is a label left from the WSL2 days)
export CYCLONEDDS_URI=file://$HOME/Autonom-cpp/install/autonom_bringup/share/autonom_bringup/config/cyclonedds.xml
```

## Run on the robot

```bash
# Raspberry Pi 5 (no GUI application runs on the robot)
scripts/start_robot.sh                         # wheels + rf2o, RPP + NavFn
scripts/start_robot.sh --imu                   # + BNO055 fused in the EKF
scripts/start_robot.sh --controller mppi       # RPP maps, MPPI races (see Nav2 modes)
scripts/start_robot.sh --planner thetastar     # Theta* global planner
scripts/start_robot.sh --map-only              # map/teleop only, no loop patrol

# Base station (every GUI application, RViz included, is started from start_sim.sh)
scripts/start_sim.sh --view                    # RViz on the real robot
```

Nothing moves until the race starts: publish `std_msgs/Empty` on `/race/start_signal` (or `/race/start`); the
start gate holds `/cmd_vel` for the 5 s start delay (`docs/06_competition_rules.md`), then the patrol explores the
track, freezes the map after the first lap and races. `/race/stop_signal` stops it. The start button on GPIO16
(`start_button` in `hardware.launch.py`) publishes `/race/start_signal`; an IR receiver driver is still missing.

## Simulation (base station, Gazebo Harmonic)

```bash
scripts/start_sim.sh                         # Fibonacci race arena: asks for the speeds, Gazebo GUI + RViz
scripts/start_sim.sh --speed 1.2             # race laps at 1.2 m/s, no questions
scripts/start_sim.sh --speed 0.6 controller:=mppi planner:=thetastar
scripts/start_sim.sh robot:=autonom          # the real robot's model (default: TurtleBot3 Burger)
scripts/start_sim.sh yaw:=3.1416             # race the other way round
```
## Simulation on one Machine

```bash
scripts/start_sim.sh robot:=tb3_fast --speed 1.2 --controller mppi --planner navfn
```

`start_sim.sh` wraps `ros2 launch robot_sim fibonacci_simulation.launch.py`; any of its arguments can be added
(`robot:=tb3|tb3_fast|autonom`, `controller:=`, `planner:=`, speeds, accelerations). The race starts on launch
(`auto_start`). The older `circuit2` track: `ros2 launch robot_sim tb3_simulation.launch.py`.

- The simulated LiDAR matches the RPLiDAR S3 (3200 samples/rev at 10 Hz, 0.1125°, 0.05–40 m) and the wheel
  odometry counts at the real encoder resolution (1400 counts/rev), so `slam_toolbox.yaml` and the EKF are shared.
- Gazebo DiffDrive's TF is bridged to `/sim/tf`, not `/tf`; the sim IMU goes through the same covariance relay as
  the BNO055 driver's values.
- `robot_sim/config` only layers TurtleBot3 differences on the robot's `autonom_bringup/config`.

## Nav2 modes

`controller:=` and `planner:=` select the path follower and global planner, on the robot (`bringup.launch.py`) and
in the simulation (`fibonacci_simulation.launch.py` / `start_sim.sh`). Simulation, TurtleBot3, laps in the 95 s
after fast mode (details: [`docs/SESSION_2026-11-09.md`](docs/SESSION_2026-11-09.md)):

| Mode | 0.6 m/s | 1.2 m/s | Notes |
|---|---|---|---|
| `controller:=mppi` (RPP maps, MPPI races) | **7 laps**, 12.5–13.8 s | **8 laps**, 11.0–13.0 s | keeps its speed in turns; MPPI CPU on the Pi untested |
| `planner:=thetastar` (RPP) | 6 laps | 8 laps | gets "goal in lethal cost" refusals at times (open issue) |
| default: RPP + NavFn | 5 laps, never < 0.17 m from a wall | 6 laps | most conservative |
| `planner:=smac` (RPP) | 4 laps, 2 failed | 6 laps | many collision stalls |
| `controller:=graceful` (sim only) | 2–3 laps | 2–4 laps | slow in every turn |

The theoretical best lap (optimal racing line at the robot's limits) is ~5.3 s: [`docs/racing_line`](docs/racing_line).

## Bring-up checklist (real robot)

1. Verify the wiring against the [pinout](#pinout-raspberry-pi-5), the `pwm_chip` index (`ls /sys/class/pwm`) and `ticks_per_revolution` (1400) in `autonom_hardware/config/hardware.yaml`.
2. With wheels off the ground, check `/joint_states` velocities are positive when driving forward; fix with `*.inverted`.
3. Tune `pid.kf` first (effort per rad/s, ~1 / 31.42), then `pid.kp` / `pid.ki` live: `ros2 param set /motor_driver pid.kp 0.05`.
4. Check the LiDAR yaw (`laser_yaw` in the URDF) so the scan lines up with the robot's front in RViz.
5. Check the BNO055 gyro before fusing it (`docs/07_imu_gyro.md`).

## Docs

| File | Content |
|---|---|
| `docs/01`–`05` | Hardware, software architecture, network, git workflow, simulation |
| `docs/06_competition_rules.md` | Start delay, wall contact, lap scoring |
| `docs/07_imu_gyro.md` | BNO055 gyro specs and checks |
| `docs/HANDOFF.md` | Current state, decisions to keep, open issues |
| `docs/REPORT.md` | Race results and to-do list |
| `docs/SESSION_*.md` | Per-session measurements and changes |

Branches: work goes to `not_tested` and `optimised` (CPU optimisations, C++ patrol); never commit to `main`.
