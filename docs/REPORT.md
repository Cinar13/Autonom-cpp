# Race readiness report: 2026-11-07

Branch `not_tested`. Everything below was verified in Gazebo Harmonic (RTX 4050) unless marked otherwise.
Nothing has run on the real robot yet. Details per change are in `docs/HANDOFF.md`, section 1.

## Result

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
