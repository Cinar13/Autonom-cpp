#!/usr/bin/env bash
# Base station: every GUI application, no robot code. The simulated Fibonacci arena (Gazebo Harmonic GUI + the
# simulated robot + RViz), or with --view only RViz, watching the real robot (started with scripts/start_robot.sh).
# The robot code (EKF, slam_toolbox, Nav2, loop patrol, start gate) runs separately, in a second terminal:
#   scripts/start_robot.sh --sim               against this simulation (asks for the speeds and the Nav2 mode)
#   scripts/start_robot.sh                     on the Pi
#
#   scripts/start_sim.sh                       the simulation, robot:=tb3_fast
#   scripts/start_sim.sh --view                RViz on the real robot, no simulation
#   scripts/start_sim.sh robot:=autonom        the real robot's model (start_robot.sh --sim robot:=autonom with it);
#   scripts/start_sim.sh yaw:=3.1416           any fibonacci_simulation argument can be added, e.g. race the other
#                                              way round
#
# Options:
#   --view                                     RViz only, for the real robot (wall-clock time, no Gazebo)
# Gazebo's wheel limits are opened wide; the speeds and accelerations are chosen in start_robot.sh --sim.
# Needs a built workspace (scripts/install.sh). Ctrl+C stops everything.
set -euo pipefail

view=no
launch_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --view) view=yes; shift ;;
    -h | --help) sed -n '2,18p' "$0"; exit 0 ;;
    --speed | --race-speed | --mapping-speed | --mapping-accel | --race-accel | --turn-speed | --controller | --planner | \
      --defaults | controller:=* | planner:=* | *_speed:=* | *_accel:=*)
      echo "$1 belongs to the robot code now: scripts/start_robot.sh --sim $1 ..." >&2
      exit 1 ;;
    *) launch_args+=("$1"); shift ;;
  esac
done

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

# Hybrid-graphics laptops in on-demand mode: render Gazebo and RViz on the NVIDIA GPU (docs/HANDOFF.md section 2).
if command -v prime-select >/dev/null && [[ "$(prime-select query 2>/dev/null)" == "on-demand" ]]; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi

if [[ "$view" == yes ]]; then
  echo "Starting: ros2 launch autonom_bringup view.launch.py ${launch_args[*]}"
  exec ros2 launch autonom_bringup view.launch.py "${launch_args[@]}"
fi

if pgrep -f '(^|/)gz sim( |$)' >/dev/null; then
  echo "A Gazebo simulation is already running. Stop it first (Ctrl+C in its terminal, or: pkill -f 'gz sim')." >&2
  exit 1
fi

# A stale ROS 2 CLI daemon can hide topics from `ros2 topic` commands in other terminals.
ros2 daemon stop >/dev/null 2>&1 || true

robot="tb3_fast"
for arg in "${launch_args[@]}"; do
  [[ "$arg" == robot:=* ]] && robot="${arg#robot:=}"
done
case "$robot" in tb3 | tb3_fast | autonom) ;; *) echo "Unknown robot '$robot' (tb3, tb3_fast, autonom)." >&2; exit 1 ;; esac

args=("stack:=world" "robot:=${robot}")
for arg in "${launch_args[@]}"; do
  [[ "$arg" == robot:=* || "$arg" == stack:=* ]] || args+=("$arg")
done
cleanup() {
  # ros2 launch can exit before the separate Gazebo server and GUI processes do, leaving them running (and blocking
  # the next start). No Gazebo was running when this script started, so any left now belongs to this session.
  local gz='(^|/)gz sim( |$)'
  pgrep -f "$gz" >/dev/null || return 0
  pkill -INT -f "$gz" || true
  for _ in 1 2 3 4 5; do
    pgrep -f "$gz" >/dev/null || return 0
    sleep 1
  done
  pkill -KILL -f "$gz" || true
}
trap cleanup EXIT

echo "Starting: ros2 launch robot_sim fibonacci_simulation.launch.py ${args[*]}"
echo "Then, in a second terminal: scripts/start_robot.sh --sim robot:=${robot}"
ros2 launch robot_sim fibonacci_simulation.launch.py "${args[@]}"
