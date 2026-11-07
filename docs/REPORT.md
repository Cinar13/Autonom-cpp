# Race readiness report: 2026-11-07

Branch `not_tested`. Everything below was verified in Gazebo Harmonic (RTX 4050) unless marked otherwise.
Nothing has run on the real robot yet. Details per change are in `docs/HANDOFF.md`, section 1.

## Result (branch `optimised`, C++ loop patrol)

Same race, same robot, with the loop patrol ported to C++ (`autonom_patrol`):

| | Python patrol | C++ patrol |
|---|---|---|
| **Valid laps in 3 min** (ground truth) | 6 | **7** (63, 79, 95, 114, 130, 147, 163 s) |
| Reverse crossings | 0 | 0 |
| loop_patrol CPU / RAM (USS) | 64 % / 85 MB | **7.5 % / 25 MB** (~5 % of it is the sim `/clock`) |
| Robot stack CPU / RAM (USS) | 346 % / 590 MB | **172 % / 461 MB** |

## Result (branch `not_tested`, Python loop patrol)

3-minute race on the Fibonacci arena (`ros2 launch robot_sim fibonacci_simulation.launch.py robot:=tb3_fast`),
timed in simulated seconds from the start gate's release, lap count from Gazebo ground truth:

| | |
|---|---|
| **Valid laps in 3 min** | **6** (start-line crossings at 63, 79, 96, 112, 128, 161 s) |
| Reverse crossings | 0 |
| Mapping lap (0.3 m/s) | 63 s incl. the 5 s start hold |
| Race laps (0.6 m/s) | 16–17 s typical, one 33 s lap |
| Collision stalls / recoveries | 19 / 1 backup, 2 controller timeouts |

Robot: TurtleBot3 body with Gazebo limits raised to the real robot's 0.6 m/s, on the real robot's tuning.

## What the stack does now

- **Start**: 5 s start gate (`autonom_race`); `/cmd_vel` is held at zero until it releases.
- **Never turns back**: a keepout wall across the track behind the robot (rear gate, global costmap
  KeepoutFilter) follows forward travel; exploration goals and laps are planned by driving distance ahead.
- **Mapping lap**: explores the track centre line at 0.3 m/s until the ring closes.
- **Race laps**: one continuous Nav2 goal per lap along the centre line at 0.6 m/s; the next lap is sent before
  the current one ends.
- **Moving robots / removed obstacles**: global costmap from live LiDAR only (speckle-filtered); the patrol
  reuses its last lap while an obstacle breaks the map.
- **Localisation**: EKF fuses wheels, gyro and rf2o (default on the arena; wheels alone drifted up to 1 rad at
  0.6 m/s); slam_toolbox maps live as `docs/06` requires.

## Key findings this session

| Problem | Cause | Fix |
|---|---|---|
| Robot stalled in corners | RPP compared the carrot distance with a goal tolerance equal to the minimum lookahead | `min_lookahead_dist` > `xy_goal_tolerance` |
| Fake loops with a 180° turn | partial map accepted as a loop | lap only when a sealed cut is reached the long way round |
| Turned back on the arena | straight-line exploration goals; gate stuck after hairpins | driving-distance centre-line goals; gate follows forward travel by robot heading |
| SLAM map rotated up to ~1 rad | `occupancy_threshold` 0.4 erased the 18 mm inner walls | back to 0.1 |
| Stuck in corner pockets at 0.6 m/s | overshoot while the map was incomplete | map at 0.3 m/s, race at 0.6 m/s |
| Odometry drift at speed | wheel slip | gyro + rf2o fused |
| Patrol never sent goals | `waitUntilNav2Active` without timeout | own wait with retries |

## CPU and memory (robot side)

Measured on the development laptop (Intel i7-12650H, 16 threads) during 60 s of racing on the arena, Gazebo,
RViz, the ros_gz bridge and the sim-only wheel relay excluded. 100 % = one core; USS = memory unique to the process.

| Process | CPU avg | CPU peak | RAM (USS) |
|---|---|---|---|
| slam_toolbox | 125 % | 306 % | 117 MB |
| loop_patrol (Python) | 64 % | 68 % | 85 MB |
| rf2o_laser_odometry | 24 % | 26 % | 21 MB |
| bt_navigator | 17 % | 18 % | 48 MB |
| planner_server | 15 % | 16 % | 28 MB |
| controller_server | 14 % | 16 % | 27 MB |
| ekf_filter_node | 11 % | 13 % | 20 MB |
| behavior_server | 11 % | 13 % | 21 MB |
| 11 small nodes together | 68 % | | 219 MB |
| **Total (19 processes)** | **~346 %** | | **~590 MB** |

In simulation every node also handles `/clock` at 1 kHz (~5 % each, ~0.9 core in total); that load does not exist
on the robot. Scaled to the Raspberry Pi 5 (4 cores, roughly 2–2.5× slower per core) the remaining ~2.5 laptop
cores would still exceed the Pi: slam_toolbox and the Python loop patrol are the two big consumers.

**After the C++ patrol (branch `optimised`), same measurement:** total **172 % / 461 MB**. loop_patrol 7.5 % /
25 MB; slam_toolbox 18 % avg, 28 % peak / 59 MB. The slam_toolbox drop is not from the port: the measurement
above was taken with `occupancy_threshold: 0.4`, which erased the arena's inner walls and left SLAM fighting a broken
map; with 0.1 it converges cheaply. Without the ~0.9 core of sim `/clock` overhead the stack needs roughly
0.8 laptop cores, about 2 Pi 5 cores: it should fit, to be confirmed on the Pi.

## To do

Ordered by expected effect on the score.

1. **SLAM map degrades after ~2.5 min at race speed.** In the last race the SLAM pose showed false reverse
   crossings from 151 s while the robot (ground truth) still lapped forward. Measure it with the recorder
   (map + scan snapshots, ground truth) and look at loop-closure behaviour over many laps.
2. **Lap counter undercounts** (2 counted vs 6 driven). It depends on the SLAM pose and the wall-aware start
   zone; once 1 is fixed re-check, or count laps on odometry-corrected crossings.
3. **One slow lap (33 s) per race.** Find what stalls it (snapshots around the lap); RPP collision stalls are the
   usual suspect at hairpins.
4. **Real robot model** (`robot:=autonom` or the coming smaller model): race the full 3 minutes in the sim;
   set `robot_radius`, `inflation_radius`, patrol `clearance`, wheel geometry (URDF + `hardware.yaml`).
5. **Moving rivals**: the two-cylinder harness was only written for the old circuit; port it to the arena and
   race against 2–4 rivals.
6. **Real hardware bring-up** (`docs/HANDOFF.md` §4.5): pins, PWM chip, encoder ticks, PID, LiDAR yaw, permissions.
7. **Start signal driver**: button / RC5 IR publishing `/race/start_signal` (pin not in `docs/01`).
8. **Official rule numbers** for the team rule "no reverse crossing of the start" (`docs/06` §4).
9. CPU check on the Raspberry Pi 5 with rf2o, gyro, SLAM and Nav2 all running at race speed.
10. ~~Port the loop patrol to C++~~ **done on `optimised`**: `autonom_patrol` (planner library + event-driven node,
    26 gtests), 64 % → 7.5 % CPU, 6 → 7 laps.
11. ~~Reduce slam_toolbox CPU~~ mostly solved by the `occupancy_threshold` fix (125 % → 18 %). Scan decimation for
    SLAM remains an option if the Pi needs it.
12. Re-measure CPU and RAM after 10 and 11, without the 1 kHz `/clock` overhead (real-time run or on the Pi).
