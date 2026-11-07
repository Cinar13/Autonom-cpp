#!/usr/bin/env bash
# Start the race simulation: Gazebo Harmonic (GUI) + RViz + the full autonomy stack on the Fibonacci arena.
#
#   scripts/start_sim.sh                     TurtleBot3 at the real robot's speed (robot:=tb3_fast)
#   scripts/start_sim.sh robot:=autonom      the real robot's model; any fibonacci_simulation argument can be added,
#   scripts/start_sim.sh yaw:=3.1416         e.g. race the other way round
#
# The workspace must be built (scripts/install.sh does that). Ctrl+C stops everything.
set -euo pipefail

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

if pgrep -f "gz sim" >/dev/null; then
  echo "A Gazebo simulation is already running. Stop it first (Ctrl+C in its terminal, or: pkill -f 'gz sim')." >&2
  exit 1
fi

# Hybrid-graphics laptops in on-demand mode: render Gazebo and RViz on the NVIDIA GPU (docs/HANDOFF.md section 2).
if command -v prime-select >/dev/null && [[ "$(prime-select query 2>/dev/null)" == "on-demand" ]]; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi

# A stale ROS 2 CLI daemon can hide topics from `ros2 topic` commands in other terminals.
ros2 daemon stop >/dev/null 2>&1 || true

args=("$@")
if [[ ! " ${args[*]-} " =~ " robot:=" ]]; then
  args=("robot:=tb3_fast" "${args[@]}")
fi
echo "Starting: ros2 launch robot_sim fibonacci_simulation.launch.py ${args[*]}"
exec ros2 launch robot_sim fibonacci_simulation.launch.py "${args[@]}"
