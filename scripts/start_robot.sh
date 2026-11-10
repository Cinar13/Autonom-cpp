#!/usr/bin/env bash
# Start the full stack on the robot (Raspberry Pi 5): hardware drivers + EKF + slam_toolbox + Nav2 + loop patrol.
# No GUI application runs here; watch the robot from the base station with: scripts/start_sim.sh --view
#
#   scripts/start_robot.sh                                   wheels + rf2o, RPP + NavFn
#   scripts/start_robot.sh --controller mppi                 RPP maps, MPPI races (check its CPU on the Pi)
#   scripts/start_robot.sh --planner thetastar --imu         Theta* planner, BNO055 fused in the EKF
#   scripts/start_robot.sh --map-only                        no loop patrol (map / teleop only)
#   scripts/start_robot.sh lidar_serial_port:=/dev/ttyUSB1   any bringup.launch.py argument can be added
#
# Options:
#   --controller   rpp (default) | mppi
#   --planner      navfn (default) | smac | thetastar
#   --imu          start the BNO055 driver and fuse /imu/data (use_imu:=true)
#   --no-rf2o      wheel odometry only, no rf2o laser odometry (use_rf2o:=false)
#   --map-only     do not run the loop patrol (autonomous_loop:=false)
# Nothing moves until the race starts: publish std_msgs/Empty on /race/start_signal; the start gate then holds
# /cmd_vel for the 5 s start delay (docs/06_competition_rules.md). Needs a built workspace. Ctrl+C stops everything.
set -euo pipefail

controller="rpp"
planner="navfn"
launch_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --controller) controller="$2"; shift 2 ;;
    --planner) planner="$2"; shift 2 ;;
    --imu) launch_args+=("use_imu:=true"); shift ;;
    --no-rf2o) launch_args+=("use_rf2o:=false"); shift ;;
    --map-only) launch_args+=("autonomous_loop:=false"); shift ;;
    -h | --help) sed -n '2,19p' "$0"; exit 0 ;;
    controller:=*) controller="${1#controller:=}"; shift ;;
    planner:=*) planner="${1#planner:=}"; shift ;;
    *) launch_args+=("$1"); shift ;;
  esac
done
case "$controller" in rpp | mppi) ;; *) echo "Unknown controller '$controller' (rpp, mppi)." >&2; exit 1 ;; esac
case "$planner" in navfn | smac | thetastar) ;; *) echo "Unknown planner '$planner' (navfn, smac, thetastar)." >&2; exit 1 ;; esac

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed (/opt/ros/jazzy). Run scripts/install.sh first." >&2
  exit 1
fi
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "The workspace is not built ($WS/install missing). Run scripts/install.sh, or:" >&2
  echo "  cd $WS && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" >&2
  exit 1
fi

# setup.bash scripts read unset variables; relax -u while sourcing them.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

if pgrep -f '(^|/)gz sim( |$)' >/dev/null; then
  echo "A Gazebo simulation is running on this machine; its topics would mix with the robot's. Stop it first." >&2
  exit 1
fi
# Same DDS setup as the base station (README, "CycloneDDS"); without it RViz there may not see the robot.
if [[ "${RMW_IMPLEMENTATION:-}" != rmw_cyclonedds_cpp ]]; then
  echo "Warning: RMW_IMPLEMENTATION is not rmw_cyclonedds_cpp (see README, CycloneDDS)." >&2
fi

# A stale ROS 2 CLI daemon can hide topics from `ros2 topic` commands in other terminals.
ros2 daemon stop >/dev/null 2>&1 || true

args=("controller:=${controller}" "planner:=${planner}" "${launch_args[@]}")
echo "Starting: ros2 launch autonom_bringup bringup.launch.py ${args[*]}"
exec ros2 launch autonom_bringup bringup.launch.py "${args[@]}"
