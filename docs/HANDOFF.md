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
| `autonom_patrol` | C++ loop patrol (branch `optimised`): `track_planner` library + `LoopPatrolNode` component (`loop_patrol` executable); 26 gtests |
| `autonom_race` | C++ `RaceStartGateNode` + ROS-free `StartGate` state machine (IDLE → COUNTDOWN → RACING / STOPPED); 7 gtests |
| `turtlebot3_description` | Vendored Burger-only copy (ROBOTIS jazzy `0c0be84`, Apache-2.0) |
| `rf2o_laser_odometry`, `sllidar_ros2` | Git submodules, unmodified |

### Data flow
```
                        [use_rf2o] /scan -> scan_decimator -> rf2o -> covariance relay -> /rf2o/odom --+
real: encoders -> wheel_odometry -> /wheel/odom                                                        |
sim:  Gazebo /odom -> covariance relay -> /wheel/odom ---------------------------------------------- +--> EKF -> /odometry/filtered
                        [use_imu]  BNO055 or Gazebo IMU -> /imu/data ------------------------------------+    + TF odom->base_footprint
EKF inputs: wheels vx + vy (= 0) + vyaw, rf2o vx only, IMU vyaw + ax (all velocities / accelerations, no poses)
/scan -> slam_toolbox (maps, then localizes only once loop_patrol freezes the map) -> /map + TF map->odom -> Nav2 <- loop_patrol
loop_patrol -> RPP desired_linear_vel + velocity_smoother max_accel (mapping profile, fast mode 10 s after the freeze)
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
| Loop patrol in C++ (branch `optimised`) | `autonom_patrol`: `track_planner` (pure C++17 port of the planner: safe space, driving distance, Felzenszwalb distance transform, sealed rear gate, laps, exploration, start line / wall-aware zone; 26 gtests) and `LoopPatrolNode` (event-driven 20 Hz state machine, Nav2 action clients, non-blocking speed parameter client, TF on its own thread). Same parameters and log lines as the Python version, which was removed. Race: 7 laps in 3 min (Python 6), patrol CPU 64 % → 7.5 %, stack 346 % → 172 %. |
| Map slowly, race fast | `loop_patrol` sets RPP `FollowPath.desired_linear_vel` live: `exploration_speed` 0.3 m/s while mapping, `race_speed` 0.6 m/s in fast mode (TurtleBot3 overlays: 0.22 / 0.22). At full speed the mapping lap overshot hairpins into corner pockets and got stuck. Since `f0b02ea` fast mode starts `race_speed_delay` (10 s) after the map freeze, not when the lap is found (see below). |
| Rear gate follows forward travel | The gate advances when the displacement agrees with the robot's heading (`cos > 0.5`), every 0.2 m. Comparing with the gate's previous direction failed in the hairpins (180° in ~0.5 m) and sent the robot back. |
| EKF inputs changed (`d37c4a1`, user decisions) | `ekf_imu.yaml` also fuses the IMU's forward acceleration `ax` (6-axis IMU is enough; gravity removed, `imu0_linear_acceleration_rejection_threshold` 3.0; sim accel noise 0.017 m/s²). `ekf_rf2o.yaml` fuses rf2o's `vx` only, not its yaw rate: heading comes from gyro + wheels. Removing rf2o entirely (`use_rf2o:=false`) was measured and is worse: SLAM error up to 1.26 m / ~1.1 rad, 179 collision stalls, 3 failed laps, 1 real reverse crossing. Keep `use_rf2o:=true`. |
| Map freeze after the mapping lap (`f0b02ea`) | `autonomy.launch.py localize_after_mapping:=true` (default) runs slam_toolbox's `map_and_localization_slam_toolbox_node` (same mapping code path as the async node; still named `slam_toolbox`). At the first forward start-line crossing after a lap is found (`loop_patrol.localize_after_laps` 1), the patrol calls `/slam_toolbox/set_localization_mode` (`std_srvs/SetBool`): from then on scans are only matched against the map built so far (`ProcessLocalization`) and the map stops growing, so rivals cannot warp it. `docs/06` is still met: slam_toolbox mapped the loop, no prebuilt map. Log: `Map frozen: slam_toolbox now localizes by scan matching against it`. Effect: SLAM error stays flat for the rest of the race (0.31–0.39 m max in 5 runs) instead of growing past 1 m. `localize_after_mapping:=false` restores `async_slam_toolbox_node` and mapping all race. |
| Gentle mapping, delayed fast mode (`f0b02ea`) | `loop_patrol` also sets `velocity_smoother.max_accel` live: `exploration_max_accel` [0.5, 0, 2.0] while mapping, `race_max_accel` [1.5, 0, 5.0] in fast mode (`nav2_sim.yaml`: [1.0, 0, 3.0]). Fast mode starts `race_speed_delay` 10 s (node clock) after the map freeze; laps before that run at the mapping profile. Logs: `Map finished: fast mode in 10.0 s`, then `Fast mode`. Gentler mapping acceleration did not measurably reduce the error that builds up during the mapping lap (0.41 m vs 0.37 m). |
| Stress test (`f0b02ea`, GUI, ground truth) | Three 3-minute races on the arena: default direction 6 race laps (15.5–25.4 s, 1 failed lap recovered, 133 stalls); reverse (`yaw:=3.1416`) 7 laps (15.7–17.4 s, 0 failures, 26 stalls); 7 cm post 0.2 m off the centre line of the right straight for 30 s after the freeze, 7 laps, passed twice, closest 2 cm, no contact, map error unchanged after removal. 0 reverse crossings in all runs. The patrol's lap count now matches ground truth. |
| Map rotated at the first hairpin (fixed) | Not a timing problem: measured in sim, every input arrives within ~11 ms of its stamp (scan 11 ms, odometry/EKF < 3 ms, IMU < 1 ms), scans line up with ground truth at a 0.000 s offset even while turning, no slam_toolbox TF/drop warnings, EKF yaw within 0.01 rad of ground truth. Cause: slam_toolbox's heading penalty was effectively off (`angle_variance_penalty` 1.0, `minimum_angle_penalty` 0.9: 0.05 rad off odometry cost 0.05 %), so the matcher slid in yaw in the round hairpins, mapped the first one ~0.2 rad rotated, and the frozen map kept the bend (SLAM yaw error up to 0.47 rad every lap). Now 0.001 / 0.5. 150 s, ground truth: SLAM yaw max 0.47 → 0.12 rad, position max 0.39 → 0.08 m, collision stalls 148 → 22, failed goals 13 → 0. |
| Section speed caps (`f47b6cc`) | Above 1 m/s the robot overshot the arena's corners (Nav2 is tuned for 0.6 m/s). `loop_patrol` now splits the lap into straights where it turns by more than `corner_angle` (0.35 rad) at a waypoint and, in fast mode, sets the cruise speed from the robot's nearest lap segment: straights shorter than `short_straight_length` 0.8 m at `short_straight_speed` 1.0 m/s, and the last `corner_approach_distance` 0.5 m before every corner at `corner_approach_speed` 1.0 m/s (a cap only slows a faster robot, so it brakes for corners only above 1 m/s). On the arena 12–14 of ~17 lap segments end up capped. `robot:=tb3_fast`, race speed 1.4 m/s, 95 s after fast mode (GUI): no caps 3 laps (21.6, 21.6, 49.0 s), 3 failed, 1 reverse crossing, 300 collision stalls, a wall within 0.15 m 25 % of the time; short-straight cap only 2 laps, 4 failed, 431 stalls, 17 %; both caps 5 laps (18.6, 40.3, 12.6, 12.1, 11.7 s), 2 failed, 0 reverse, 53 stalls, 5 %, never above 1 m/s in a turn (was 11 %). Still touches a wall now and then (LiDAR minimum 0.12 m); the 0.6 m/s Nav2 tuning (lookahead, collision horizon) is the next limit. At 1.25 m/s (same caps, same window): 4 laps (18.0, 10.1, 12.9, 11.8 s), 0 failed, 145 stalls, a wall within 0.15 m 5.7 % of the time, but **the robot turned round**: it stalled at the first corner (0.81, −0.57) facing up the right arm, the BackUp recovery reversed it 0.15 m into the corner, and Nav2 then planned back along the start straight; every later lap ran the wrong way (5 reverse crossings, 3 in the window, confirmed by ground truth). The rear gate did not block the way back after a backup in a corner: open issue. |
| Rear gate in corners; Nav2 for high speed | **Rear gate (fixed):** the gate was a line 0.25 m behind the robot across its *current* heading; in a corner that line ran along the entry corridor and left the way back open, so after a BackUp recovery at the first corner Nav2 planned back down the start straight (the 1.25 m/s turn-round above). The gate now sits on the travelled path (`trailPointBehind`: the point `rear_gate_offset` back along the robot's forward trail, across the direction of travel there). **Nav2 for speed** (`robot_sim/config/nav2_fibonacci_fast.yaml`): collision horizon 0.5 → 0.3 s, curvature braking from 0.9 m radius (was 0.6) with a 0.3 m/s floor, lookahead 0.5 s up to 0.7 m, turn on the spot only beyond 1.4 rad (was 0.785), smoother braking 3 m/s²; `loop_patrol` section caps off by default. tb3_fast, 95 s after fast mode, GUI: 1.4 m/s Nav2 tuning without the spot-turn change 6 laps (14.7–17.4 s), 0 failed, 0 reverse, 64 stalls, wall within 0.15 m 2.7 %; 1.25 m/s full tuning 7 laps (11.3–16.0 s), 0 failed, 0 reverse, 18 stalls, 0.2 %, turning on the spot 1.4 % of the time (was 20 %), stopped 3 % (was 21 %). Gazebo caps the TurtleBot3's acceleration at ~1.0 m/s². |
| Nav2 mode comparison; turn-round at speed (2026-11-09) | **BT fix:** `RemovePassedGoals` ran inside the 1 Hz `RateController` of `navigate_through_poses_race.xml`; at 1.2 m/s the robot overtook a new lap's first pose by more than its 0.7 m radius, the next replan drove back to it and every later lap ran the wrong way (1 lap, 5 reverse crossings). It now runs every BT tick: 0 reverse crossings since. **Modes** (`fibonacci_simulation.launch.py controller:=rpp|mppi|graceful planner:=navfn|smac|thetastar`), tb3_fast, 95 s after fast mode: at 1.2 m/s RPP + Theta\* 8 laps (11.1–13.4 s), RPP + NavFn 6, RPP + Smac 2D 6 (92 stalls), MPPI + NavFn 5 (17.9–21.1 s, best clearance), Graceful 2–4 with failures; at 0.6 m/s RPP + Theta\* 6, RPP + NavFn 5 (0 stalls, never < 0.17 m from a wall), RPP + Smac 2D 4 (2 failed, 242 stalls). **MPPI fixed** (did not drive: asymmetric acceleration limits; never mapped: critic thresholds vs 0.5 m exploration goals; crawled: `wz_std` 0.2). Details: `docs/SESSION_2026-11-09.md`. |

Missing: a driver for the physical start button / RC5 IR receiver. It only has to publish `std_msgs/Empty` on `/race/start_signal` (and `/race/stop_signal`); pin and receiver are not in `docs/01`.

Latest race result, findings and the to-do list: **`docs/REPORT.md`**.

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

7. **Slow laps and collision stalls in the default (+x) direction.** With the map frozen, races facing +x had 22–25 s laps and 130–200 RPP "collision ahead" stalls; the reverse direction had steady 16 s laps and 26 stalls. Something on the track is tight that way round, probably a hairpin. Find the spot from the ground-truth path, then loosen RPP's braking (`regulated_linear_scaling_min_radius` 0.6) and its wall-distance slowdown (`cost_scaling_dist` 0.2 is always active in 0.6 m corridors). This is also the user's "faster turns" request, not started yet.
8. **Map error left over from the mapping lap.** The freeze keeps the 0.3–0.4 m error built up during the mapping lap (mostly after ~40 s). Laps still work because the robot localizes consistently against the frozen map; candidates to reduce it: stricter loop-closure matching (`loop_match_minimum_response_*`, `loop_search_maximum_distance` 3.0 on a 2.5 m arena) or smaller `minimum_travel_distance` / `heading`.

Harmless log noise you can ignore: rf2o `Waiting for laser_scans....` (it loops at 20 Hz on a 10 Hz scan), RViz `indexed_8bit_image` GLSL link error, Gazebo `Ogre material scripts` warnings (the circuit model's walls render grey), `gz_frame_id ... not defined in SDF`.

---

## 5. Decisions — keep these unless there is a reason to change them

| Decision | Why |
|---|---|
| EKF is the **only** publisher of `odom->base_footprint`; Gazebo DiffDrive TF is bridged to `/sim/tf`, not `/tf` | Two publishers of one TF edge makes the tree jump between them |
| Gazebo `/odom` and rf2o odometry pass through `OdometryCovarianceRelay` | Both publish all-zero covariance, which the EKF treats as perfect certainty |
| EKF fuses **velocities only** (vx, vy=0, vyaw), plus the IMU's forward acceleration `ax` | Every source drifts in absolute pose; fusing poses gives competing estimates |
| rf2o feeds the EKF `vx` only, not its yaw rate | User decision (2026-11-07); heading comes from gyro + wheels. rf2o stays on: without it the map drifted past 1 m |
| slam_toolbox freezes the map after the mapping lap (`localize_after_mapping`) | Drift stops growing and rivals cannot corrupt the map; slam_toolbox still maps the loop as `docs/06` requires |
| Simulated LiDAR stays at 10 Hz (not 40–50 Hz) | The real RPLiDAR S3 scans at 10 Hz; a faster sim sensor would give results that do not carry over to the robot |
| Simulation tests run **with the Gazebo GUI and RViz** | User request; scripted measurement runs too |
| rf2o and IMU are EKF **overlays** (`ekf_rf2o.yaml`, `ekf_imu.yaml`) | `use_rf2o` / `use_imu` add or remove them cleanly |
| One `robot_urdf` argument drives both robot_state_publisher and the Gazebo spawn; URDFs get Gazebo plugins from `gz_plugins.xacro` with `sim:=true` | Swapping the robot is a single argument, and every robot shares `bridge.yaml` |
| Simulated LiDAR = RPLiDAR S3 (3200 samples/rev at 10 Hz = 32 kHz, 0.1125°, 0.05–40 m) | User request; `slam_toolbox.yaml` is shared unchanged with the real robot |
| `real_time_factor` changed 2.0 → 1.0 in `circuit2_harmonic.sdf` (circuit model untouched) | RTF 2 doubled every rate the stack had to keep up with |
| Gazebo server and GUI run as **separate processes** | A combined `gz sim` aborted under WSL's d3d12 driver; harmless on native |
| JointStatePublisher capped at 50 Hz | It defaulted to physics rate (~1 kHz) |
| `cmd_vel` is unstamped `Twist` | Nav2 Jazzy default; the bridge and motor driver match it |
| Loop patrol drives to the map frontier when no loop fits | Without it, the robot never moved when the visible map was small |
| Nav2 uses Regulated Pure Pursuit, not MPPI; own `navigation.launch.py` without docking | Saves CPU on the Pi; Jazzy's stock launch needs docking config. Measured 2026-11-09: RPP laps fastest of RPP / MPPI / Graceful; RPP + Theta\* led at 0.6 and 1.2 m/s (one run each; NavFn stays the default until repeated) |
| libgpiod **1.6** API (not 2.x) | Ubuntu 24.04 ships 1.6.3; CMake fails fast on 2.x |

---

## 6. Git rules (from `docs/04_git_workflow.md` and the user)

- Work and push only on `not_tested` (or `not_tested/<feature>`) or `optimised`. **Never commit or push to `main`.** Push with an explicit refspec: `git push origin not_tested:not_tested` / `git push origin optimised:optimised`.
- `optimised` (from `not_tested` at `1711136`, 2026-11-07): CPU optimisations for the Raspberry Pi 5, first the loop patrol ported to C++ (`docs/REPORT.md` to-do 10–12).
- Identity must match local `git config` (Çınar Öztürk). Set it on the new machine before committing.

---

## 7. Things that do not travel with the repo

- Claude Code memory on the WSL machine (the push policy above, now written down here instead).
- Earlier simulation attempts in `~/last_code`, `~/last_code_2` and `~/last_code_3` on the WSL machine (bash-script based; the source of the spawn pose `x=0, y=-4.7, yaw=-1.5708`). Copy them over only if you want them for reference.
- `~/.ssh/id_ed25519` (see section 6).
