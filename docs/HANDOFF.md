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
| `autonom_localization` | C++ components `OdometryCovarianceRelay` (adds covariance to rf2o and Gazebo odometry), `ScanDecimator` (every Nth beam for rf2o), `ScanSpeckleFilter` (drops isolated returns before the costmaps) |
| `autonom_description` | Custom robot URDF (`autonom.urdf.xacro`, `sim:=true` adds Gazebo plugins) + `gz_plugins.xacro` macros, incl. `gz_rplidar_s3` |
| `autonom_bringup` | `bringup` = `hardware` + `autonomy` launches; EKF / SLAM / Nav2 / CycloneDDS / RViz config; `loop_patrol.py` + `loop_planner.py` (7 pytest tests) |
| `robot_sim` | `tb3_simulation.launch.py`, `circuit2_harmonic.sdf`, `bridge.yaml`, `nav2_sim.yaml`, vendored `Circuit_ql_2` model |
| `autonom_race` | C++ `RaceStartGateNode` + ROS-free `StartGate` state machine (IDLE → COUNTDOWN → RACING / STOPPED); 7 gtests |
| `turtlebot3_description` | Vendored Burger-only copy (ROBOTIS jazzy `0c0be84`, Apache-2.0) |
| `rf2o_laser_odometry`, `sllidar_ros2` | Git submodules, unmodified |

### Data flow
```
                        [use_rf2o] /scan -> scan_decimator -> rf2o -> covariance relay -> /rf2o/odom --+
real: encoders -> wheel_odometry -> /wheel/odom                                                        |
sim:  Gazebo /odom -> covariance relay -> /wheel/odom ---------------------------------------------- +--> EKF -> /odometry/filtered
                        [use_imu]  BNO055 or Gazebo IMU -> /imu/data ------------------------------------+    + TF odom->base_footprint
/scan -> slam_toolbox -> /map + TF map->odom -> Nav2 <- loop_patrol
/scan -> scan_speckle_filter -> /scan_filtered -> Nav2 costmaps
Nav2 -> /cmd_vel_smoothed -> race_start_gate -> /cmd_vel -> motor_driver (real) or Gazebo DiffDrive (sim)
race_start_gate -> /race/state (latched) -> loop_patrol sends goals only while RACING
```

### Competition rules (docs/06) — added 2026-11-07 on native Ubuntu
| Rule | Implementation |
|---|---|
| 5 s start delay (AC-3.2, AC-4.5) | `race_start_gate`: start via `/race/start_signal` (std_msgs/Empty, for a button / RC5 IR driver) or `ros2 service call /race/start std_srvs/srv/Trigger`; stop via `/race/stop_signal` or `/race/stop`. Holds zero twists until `start_delay` (5.0 s, node clock) has elapsed; repeated starts are ignored; a start before `/clock` runs is rejected. `auto_start:=true` (sim default) starts on launch; the real robot defaults to `false`. |
| No Nav2 goals before the delay | `loop_patrol` waits for `/race/state == RACING` and cancels its goal when the state leaves RACING. |
| Wall contact ≤ 3 s, no ramming (AC-3.5, AC-4.8) | `nav2.yaml`: `PoseProgressChecker` 2 s / 0.10 m / 0.35 rad; hard margin robot_radius + 20 mm (user decision) plus a steep soft band (`inflation_radius` 0.35 sim / 0.40 real, `cost_scaling_factor` 10; RPP `inflation_cost_scaling_factor` matches). With 20 mm only, the fastest path hugged inside corners and RPP's collision check stalled (218 "collision ahead", 1 failed lap); with the soft band: 0 stalls, 0 failed laps, 118 s laps; RPP cost-regulated slow-down near obstacles. `behavior_trees/navigate_to_pose_race.xml`: recovery = clear local costmap + back up 0.15 m, then clear costmaps, then wait 1 s; no Spin. |
| No U-turns in corridors | `loop_planner.plan_loop` returns a loop only when every sector around the centroid has a waypoint (closed track mapped); until then exploration goals are chosen ahead of the robot's heading. Before this, a "loop" over a partial (L-shaped) map put the next waypoint behind the robot, and RPP's rotate-to-heading oscillated around ±π: the cause of the endless `Failed to make progress` in open issue 1. |
| Robot stalled in turns ("can't take corners") | Cause, verified in sim: RPP compares the *carrot* distance with `xy_goal_tolerance` (0.25). A slow robot clamps the carrot to `min_lookahead_dist` (was 0.25) and the interpolated carrot sat at 0.2499..., which latched (`stateful: true`) rotate-to-goal-heading with vx = 0 while the goal was metres away. Fix: `min_lookahead_dist: 0.35` (must stay > `xy_goal_tolerance`). Measured live: rotate-to-heading 363/366 commands → 0/446, mean speed 0.19 m/s, first corner taken. |
| Single LiDAR points became lethal obstacles | `autonom_localization/ScanSpeckleFilter`: `/scan` → `/scan_filtered` (returns with < 2 neighbours within ±3 beams at a similar range become +inf); both costmap obstacle layers use `/scan_filtered`, slam_toolbox keeps `/scan`. 5 gtests. |
| No reverse crossing of the start (team rule, docs/06 §4) | `loop_patrol` records the start line when the race starts; `direction: auto` orients laps with the start heading; laps begin at the first waypoint ahead; exploration goals ≤ 2.5 m. `StartLineMonitor` (0.10 m hysteresis, start zone only) logs `Start line crossed forward: N lap(s) completed` or a REVERSE error. Verified in sim: forward crossing logged, no reverse. |
| Patrol never sent goals on some launches | `BasicNavigator.waitUntilNav2Active()` waits on `get_state` without a timeout and hung when `bt_navigator` dropped the first response. Replaced by `LoopPatrol.wait_until_active` (2 s timeout, retry). |
| Stale robot pose in the patrol | Its TF listener shared the node with `/clock` (1 kHz in sim) under `spin_once`, so poses lagged by seconds (missed/false start-line crossings). The listener now has a private node and thread. |
| Fastest route | Each lap is one `NavigateThroughPoses` goal (`behavior_trees/navigate_through_poses_race.xml`: continuous path, 1 Hz replanning, race recoveries); the next lap preempts it when `preempt_poses_remaining` (1) poses are left, so the robot never stops. Waypoints sit on the track centre line (`centre_line`, median radius per sector), letting the shortest path cut corners; `lap_pause` 0; RPP `regulated_linear_scaling_min_radius` 0.9 → 0.5; sim speed 0.22 m/s (TB3 max). Lap times are logged at the start line. |
| Fake loops on a partial map | `plan_loop` requires the reachable track to enclose its centroid (`require_closed_track`, flood fill must not reach the grid border); a U/L-shaped partial map used to give a loop with a 180° turn. Verified in sim: forward exploration all the way round, then Lap 1. |
| Never turn back (user requirement) | Rear gate: `loop_patrol` publishes `/rear_gate_mask`, a lethal wall across the track 0.4 m behind the robot that advances with forward travel (never flips on backing up); the global costmap `KeepoutFilter` (+ `costmap_filter_info_server`) cuts the ring behind the robot, so no plan can turn back. If the way forward is blocked, Nav2 fails and recovers in place. Exploration never falls back to goals behind the robot; lap waypoints < 2 m behind are dropped. |
| Removed obstacles / moving robots stayed in the map | Global costmap no longer has a static layer: live speckle-filtered LiDAR only (30 × 30 m window around the start, marking 8 m / clearing 9 m), so a departed robot is cleared as soon as a ray passes through. slam_toolbox `occupancy_threshold` 0.1 → 0.4, so `/map` (patrol topology) clears too. Loops are reused while an obstacle breaks the ring. |
| Robot wedged against an obstacle | Race BTs: BackUp 0.15 m, then Spin 0.8 rad (round footprint: turning in place cannot collide), clear costmaps, wait 1 s. |
| Race arena (Fibonacci-Autonomous-Map) | `robot_sim/models/Fibonacci_arena` (vendored STL from `Cinar13/Fibonacci-Autonomous-Map@fa04600`), `worlds/fibonacci_harmonic.sdf`, `config/nav2_fibonacci.yaml`, `launch/fibonacci_simulation.launch.py` (`ros2 launch robot_sim fibonacci_simulation.launch.py`, spawn bottom corridor facing +x; `yaw:=3.1416` races the other way). 2.49 × 2.10 m, ~0.6 m corridors, hairpins under the divider and round both U arms. Verified: forward exploration through every hairpin, laps of 32.8 s / 33.7 s, 0 reverse crossings, 0 failed laps. |
| Lap planner follows the track | `plan_loop` no longer orders waypoints by angle around a centroid (impossible on the arena, whose lap winds into the U). It cuts the track at the rear gate, floods forward (driving distance), requires the far side of a **sealed** cut to be reached the long way round (lap ≥ `min_loop_length`), and puts a waypoint every `waypoint_spacing` on the centre line (cell farthest from walls). Exploration goals use the same driving distance and centre line (`max_exploration_distance` ahead), so they never jump through a wall to a nearby corridor or into a corner pocket. |
| Real robot at 0.6 m/s on the race arena | The base `nav2.yaml` / `loop_patrol.yaml` are now tuned for the real robot (r = 0.15 m) on the Fibonacci arena at 0.6 m/s: short speed-scaled lookahead (0.3–0.6 m), braking for hairpins (min radius 0.6 m), 0.5 s collision horizon, 0.25 m soft margin, 8 × 8 m global costmap, arena patrol values. `robot_sim/config/nav2_sim.yaml` restores the circuit2 values for the TurtleBot3; `nav2_fibonacci.yaml` is the TurtleBot3 on the arena. `fibonacci_simulation.launch.py robot:=autonom` races the real robot's model (Gazebo cap raised to 0.65 m/s). A TurtleBot3 with its Gazebo limits temporarily raised (not committed) lapped the arena in 16.2 s / 17.8 s with this tuning. |
| Fast TurtleBot3 (`robot:=tb3_fast`) | `urdf/turtlebot3_burger_fast.urdf.xacro` (Gazebo limits 0.65 m/s, 4 rad/s) + `config/nav2_fibonacci_fast.yaml` (TurtleBot3 body on the base 0.6 m/s tuning). Lower soft margins (0.14 m inflation, 0.10 m clearance, 0.3 s collision horizon) made the plan hug the walls: 177 RPP "collision ahead" stalls and three failed laps in a row; reverted to 0.18 m / 0.12 m / 0.5 s. |
| Arena sims fuse gyro + rf2o by default | `fibonacci_simulation.launch.py` now defaults to `use_imu:=true use_rf2o:=true`. 3-minute race, `robot:=tb3_fast`: odometry error mean 0.06 m / max 0.17 m (wheels only at 0.6 m/s: up to 0.35 m / 1 rad), 5 laps (16.7, 16.1, 69.0, 20.2 s), 21 collision stalls (was 177). The 69 s lap was SLAM losing consistency; under investigation. |
| SLAM pose drift on the arena (fixed) | Cause: slam_toolbox `occupancy_threshold` 0.4 (set for faster obstacle clearing) erased the arena's 18 mm inner walls, leaving only the near-symmetric outer box to match; back at 0.1 the map keeps the U and divider and scan, map and ground truth line up. Earlier measurements, all taken at 0.4: EKF odometry stays within ~0.1 m over 3 min, but the slam_toolbox pose drifts 0.3–0.7 m within a lap and snaps back at loop closures (no correlation with turning). Tighter scan matching and a 0.025 m map both made it worse and were reverted. A prebuilt map is not allowed (docs/06: slam_toolbox must map). The robot still drives the arena correctly; at 0.6 m/s watch for the lap waypoint count shrinking (map drift). |
| Laps in the designated direction (AC-5.1) | `loop_patrol.direction`: `counterclockwise` (default) or `clockwise`. Set it to the referee's direction before each round. |

Missing: a driver for the physical start button / RC5 IR receiver. It only has to publish `std_msgs/Empty` on `/race/start_signal` (and `/race/stop_signal`); pin and receiver are not in `docs/01`.

Latest race result, findings and the to-do list: **`docs/REPORT.md`**.

| Map slowly, race fast | `loop_patrol` sets RPP `FollowPath.desired_linear_vel` live: `exploration_speed` 0.3 m/s while mapping, `race_speed` 0.6 m/s once the lap is found (TurtleBot3 overlays: 0.22 / 0.22). At full speed the mapping lap overshot hairpins into corner pockets and got stuck. |
| Rear gate follows forward travel | The gate advances when the displacement agrees with the robot's heading (`cos > 0.5`), every 0.2 m. Comparing with the gate's previous direction failed in the hairpins (180° in ~0.5 m) and sent the robot back. |

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
| 7a | Start delay holds | launch log: `race_start_gate`: `Start (auto_start)`, then `COUNTDOWN -> RACING` ≥ 5 s later; `ros2 topic echo /race/state` | robot does not move before RACING |
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
2. **`loop_patrol.py` uses 60–100 % of a core.** (TF now has its own thread; `/clock` still lands on the patrol node.) It is a Python process with two nodes handling `/clock` (1 kHz) and `/tf`. Options: move the BasicNavigator and the planner into one node, or port the patrol to C++ (the planner is already a pure, tested module).
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
