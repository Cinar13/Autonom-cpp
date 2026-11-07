# Handoff — moving development from WSL2 to native Ubuntu (RTX 4050)

Written 2026-11-07 at the end of the WSL2 session. Branch `not_tested`, last code commit `c08701b`.
Nothing here has run on real hardware. The simulation has run, but a full lap was never observed to succeed.

---

## 1. What exists now

### Commits on `not_tested`
| Commit | Content |
|---|---|
| `4f1a705` | Real-robot workspace: C++17 A4950 motor driver + quadrature encoder odometry, EKF, slam_toolbox, Nav2, loop patrol, CycloneDDS config, rf2o/sllidar as submodules |
| `c08701b` | Gazebo Harmonic simulation (`robot_sim`), vendored TurtleBot3 Burger, RPLiDAR S3 sim sensor, `autonom_localization` package, shared autonomy launch, `use_rf2o` switch, frontier step in loop patrol |

`main` is still at `fe89cd0` (the original first commit). GitHub also has a stray branch **`not-tested` (hyphen)** pointing at `fe89cd0`. It was not created in this session and has none of this work; it can be deleted.

### Packages (`src/`)
| Package | What it is |
|---|---|
| `autonom_hardware` | C++ components `MotorDriverNode`, `WheelOdometryNode`; RAII GPIO (libgpiod **1.6** API), sysfs PWM, PID, kinematics; 12 gtests |
| `autonom_localization` | C++ components `OdometryCovarianceRelay` (adds covariance to rf2o and Gazebo odometry), `ScanDecimator` (every Nth beam for rf2o) |
| `autonom_description` | Custom robot URDF (`autonom.urdf.xacro`, `sim:=true` adds Gazebo plugins) + `gz_plugins.xacro` macros, incl. `gz_rplidar_s3` |
| `autonom_bringup` | `bringup` = `hardware` + `autonomy` launches; EKF / SLAM / Nav2 / CycloneDDS / RViz config; `loop_patrol.py` + `loop_planner.py` (7 pytest tests) |
| `robot_sim` | `tb3_simulation.launch.py`, `circuit2_harmonic.sdf`, `bridge.yaml`, `nav2_sim.yaml`, vendored `Circuit_ql_2` model |
| `turtlebot3_description` | Vendored Burger-only copy (ROBOTIS jazzy `0c0be84`, Apache-2.0) |
| `rf2o_laser_odometry`, `sllidar_ros2` | Git submodules, unmodified |

### Data flow
```
                        [use_rf2o] /scan -> scan_decimator -> rf2o -> covariance relay -> /rf2o/odom --+
real: encoders -> wheel_odometry -> /wheel/odom                                                        |
sim:  Gazebo /odom -> covariance relay -> /wheel/odom ---------------------------------------------- +--> EKF -> /odometry/filtered
                        [use_imu]  BNO055 or Gazebo IMU -> /imu/data ------------------------------------+    + TF odom->base_footprint
/scan -> slam_toolbox -> /map + TF map->odom -> Nav2 <- loop_patrol
Nav2 -> /cmd_vel -> motor_driver (real) or Gazebo DiffDrive (sim)
```

---

## 2. Setting up the new machine

Prerequisites: Ubuntu 24.04, ROS 2 Jazzy desktop, NVIDIA driver installed (`nvidia-smi` works).

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-rmw-cyclonedds-cpp python3-rosdep
git clone --recurse-submodules -b not_tested git@github.com:Cinar13/Autonom-cpp.git ~/Autonom-cpp
cd ~/Autonom-cpp
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --skip-keys cmake_modules
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --all        # expect: 22 tests, 0 failures
```

- `--skip-keys cmake_modules` is required: rf2o's `package.xml` lists an obsolete ROS 1 key its CMake never uses.
- rosdep installs `libgpiod-dev` (needed by `autonom_hardware`, even on a PC).
- **Do not export** `GALLIUM_DRIVER` / `MESA_D3D12_DEFAULT_ADAPTER_NAME` on native Ubuntu; those were WSL-only.
- Sanity check that OpenGL uses the NVIDIA GPU: `glxinfo -B | grep "OpenGL renderer"` should name the RTX 4050, not `llvmpipe`.
  On a hybrid laptop, set the NVIDIA GPU as primary (`sudo prime-select nvidia`, then reboot) or prefix the launch with
  `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`.

---

## 3. Test plan for the simulation (do these in order)

Start it:
```bash
ros2 launch robot_sim tb3_simulation.launch.py          # Gazebo GUI + RViz, TurtleBot3 Burger, rf2o off
```

| # | Check | Command | Expected |
|---|---|---|---|
| 1 | Sim runs in real time | `ros2 topic hz /clock` | ~1000 Hz (1 ms steps, RTF 1.0) |
| 2 | LiDAR matches RPLiDAR S3 | `ros2 topic echo /scan --once \| grep -E "angle_increment\|range_max"` | `0.0019635` (0.1125°), `40.0`; `ros2 topic hz /scan` ≈ 10 Hz |
| 3 | Odometry chain | `ros2 topic hz /wheel/odom /odometry/filtered` | ~30 Hz each |
| 4 | Full TF tree | `ros2 run tf2_ros tf2_echo map base_footprint` | a transform, no errors |
| 5 | SLAM + Nav2 up | `ros2 lifecycle get /slam_toolbox` and `/bt_navigator` | both `active` |
| 6 | Patrol drives | launch log: `Lap 1: N waypoint(s)` then `Goal succeeded` | robot moves along the track |
| 7 | **A full lap succeeds (never observed yet)** | launch log: `Lap 2:` appears, no `Lap 1 ended with FAILED` | — |
| 8 | Rendering is correct | look at Gazebo + RViz | walls, robot mesh, scan, map all visible, no flicker (the WSL session saw a rendering issue; cause unknown) |

Then the variants:
```bash
ros2 launch robot_sim tb3_simulation.launch.py use_rf2o:=true            # EKF also fuses rf2o
ros2 launch robot_sim tb3_simulation.launch.py use_imu:=true             # EKF also fuses the sim IMU
ros2 launch robot_sim tb3_simulation.launch.py gui:=false use_rviz:=false   # headless
ros2 launch robot_sim tb3_simulation.launch.py \
  robot_urdf:=$(ros2 pkg prefix autonom_description)/share/autonom_description/urdf/autonom.urdf.xacro   # custom robot
```
For `use_rf2o:=true`, confirm `ros2 param get /ekf_filter_node odom1` returns `/rf2o/odom`; for `use_imu:=true`, `imu0` returns `/imu/data`.

---

## 4. Open issues (prioritised)

1. **Lap completion unverified.** An earlier CPU-starved WSL run logged ~30 `Failed to make progress` from the controller and missed its 20 Hz rate. The last run (GPU, RTF 1.0) started Lap 1 cleanly but was stopped before it finished. If laps still fail on native: check `controller_server` rate warnings first, then the RPP parameters in `nav2.yaml` / `nav2_sim.yaml`, then waypoint clearance (`loop_patrol.clearance`).
2. **`loop_patrol.py` uses 60–100 % of a core.** It is a Python process with two nodes handling `/clock` (1 kHz) and `/tf`. Options: move the BasicNavigator and the planner into one node, or port the patrol to C++ (the planner is already a pure, tested module).
3. **Rendering issue reported in WSL** (Gazebo + RViz, d3d12 driver). No cause found in the logs. Re-check on native (test 8).
4. **rf2o is off by default in simulation** at the user's request, for CPU. With the decimator it measured ~38 % of a core. Re-enable (`use_rf2o:=true`) once the machine has headroom, because the real robot runs with it on.
5. **Real hardware is entirely untested.** Before the first run on the Pi:
   - Placeholder values in `autonom_hardware/config/hardware.yaml`: pins, `pwm_chip` index (`ls /sys/class/pwm`), `ticks_per_revolution`, `*.inverted`, PID gains.
   - `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4` in `/boot/firmware/config.txt`.
   - Permissions for `/dev/gpiochip*` and `/sys/class/pwm`.
   - `laser_yaw` in `autonom.urdf.xacro` (LiDAR mounting angle).
6. **CycloneDDS naming.** `cyclonedds.xml` uses `AUTONOM_WSL_IP` for the base-station peer. On native Ubuntu set it to that machine's LAN IP (the name is just a label). WSL mirrored networking no longer applies.

Harmless log noise you can ignore: RViz `indexed_8bit_image` GLSL link error, Gazebo `Ogre material scripts` warnings (the circuit model's walls render grey), `gz_frame_id ... not defined in SDF`.

---

## 5. Decisions — keep these unless there is a reason to change them

| Decision | Why |
|---|---|
| EKF is the **only** publisher of `odom->base_footprint`; Gazebo DiffDrive TF is bridged to `/sim/tf`, not `/tf` | Two publishers of one TF edge makes the tree jump between them |
| Gazebo `/odom` and rf2o odometry pass through `OdometryCovarianceRelay` | Both publish all-zero covariance, which the EKF treats as perfect certainty |
| EKF fuses **velocities only** (vx, vy=0, vyaw) | Every source drifts in absolute pose; fusing poses gives competing estimates |
| rf2o and IMU are EKF **overlays** (`ekf_rf2o.yaml`, `ekf_imu.yaml`) | `use_rf2o` / `use_imu` add or remove them cleanly |
| One `robot_urdf` argument drives both robot_state_publisher and the Gazebo spawn; URDFs get Gazebo plugins from `gz_plugins.xacro` with `sim:=true` | Swapping the robot is a single argument, and every robot shares `bridge.yaml` |
| Simulated LiDAR = RPLiDAR S3 (3200 samples/rev at 10 Hz = 32 kHz, 0.1125°, 0.05–40 m) | User request; `slam_toolbox.yaml` is shared unchanged with the real robot |
| `real_time_factor` changed 2.0 → 1.0 in `circuit2_harmonic.sdf` (circuit model untouched) | RTF 2 doubled every rate the stack had to keep up with |
| Gazebo server and GUI run as **separate processes** | A combined `gz sim` aborted under WSL's d3d12 driver; harmless on native |
| JointStatePublisher capped at 50 Hz | It defaulted to physics rate (~1 kHz) |
| `cmd_vel` is unstamped `Twist` | Nav2 Jazzy default; the bridge and motor driver match it |
| Loop patrol drives to the map frontier when no loop fits | Without it, the robot never moved when the visible map was small |
| Nav2 uses Regulated Pure Pursuit, not MPPI; own `navigation.launch.py` without docking | Saves CPU on the Pi; Jazzy's stock launch needs docking config |
| libgpiod **1.6** API (not 2.x) | Ubuntu 24.04 ships 1.6.3; CMake fails fast on 2.x |

---

## 6. Git rules (from `docs/04_git_workflow.md` and the user)

- Work and push only on `not_tested` (or `not_tested/<feature>`). **Never commit or push to `main`.** Push with an explicit refspec: `git push origin not_tested:not_tested`.
- Identity must match local `git config` (Çınar Öztürk). Set it on the new machine before committing.

---

## 7. Things that do not travel with the repo

- Claude Code memory on the WSL machine (the push policy above, now written down here instead).
- Earlier simulation attempts in `~/last_code`, `~/last_code_2` and `~/last_code_3` on the WSL machine (bash-script based; the source of the spawn pose `x=0, y=-4.7, yaw=-1.5708`). Copy them over only if you want them for reference.
- `~/.ssh/id_ed25519` (see section 6).
