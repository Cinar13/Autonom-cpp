# Autonom-cpp

ROS 2 Jazzy autonomous mapping/looping robot (Raspberry Pi 5) plus a Gazebo Harmonic simulation.
Read `docs/HANDOFF.md` first: current state, open issues, test plan and the decisions to keep.
The constraints live in `docs/01`–`05`; competition rules (5 s start delay, wall contact, lap scoring) in `docs/06`.

- Git: commit and push only to `not_tested` (`git push origin not_tested:not_tested`); never to `main`.
- rosdep: `rosdep install --from-paths src --ignore-src -y --skip-keys cmake_modules`.
- Build and test: `colcon build --symlink-install && colcon test && colcon test-result --all`.
- C++17 for nodes and hardware; Python only for launch files and `loop_patrol`.
- `/cmd_vel` is published only by `autonom_race/race_start_gate`; Nav2's smoother outputs `/cmd_vel_smoothed`.
- The EKF is the single owner of `odom->base_footprint`; never bridge Gazebo's DiffDrive TF onto `/tf`.
- `rf2o_laser_odometry` and `sllidar_ros2` are unmodified submodules; `turtlebot3_description` and
  `robot_sim/models/Circuit_ql_2` are vendored (see each `VENDORED.md`). Don't edit them in place.
