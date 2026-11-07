#!/usr/bin/env bash
# Start the race simulation: Gazebo Harmonic (GUI) + RViz + the full autonomy stack on the Fibonacci arena.
#
#   scripts/start_sim.sh                         asks for the speeds (Enter keeps the configured ones)
#   scripts/start_sim.sh --speed 0.5             race laps at 0.5 m/s, no questions
#   scripts/start_sim.sh --mapping-speed 0.25 --speed 0.5
#   scripts/start_sim.sh --defaults              configured speeds (mapping 0.3, race 0.6 m/s), no questions
#   scripts/start_sim.sh robot:=autonom          the real robot's model; any fibonacci_simulation argument can be
#   scripts/start_sim.sh yaw:=3.1416             added, e.g. race the other way round
#
# Speeds: 0.1 m/s up to the robot's tuned maximum (tb3_fast and autonom 0.6, tb3 0.22).
# The workspace must be built (scripts/install.sh does that). Ctrl+C stops everything.
set -euo pipefail

race_speed=""
mapping_speed=""
ask=yes
launch_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --speed | --race-speed) race_speed="$2"; ask=no; shift 2 ;;
    --mapping-speed) mapping_speed="$2"; ask=no; shift 2 ;;
    --defaults) ask=no; shift ;;
    -h | --help) sed -n '2,12p' "$0"; exit 0 ;;
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

robot="tb3_fast"
for arg in "${launch_args[@]}"; do
  [[ "$arg" == robot:=* ]] && robot="${arg#robot:=}"
done
case "$robot" in
  tb3) max_speed=0.22 ;;
  tb3_fast | autonom) max_speed=0.6 ;;
  *) echo "Unknown robot '$robot' (tb3, tb3_fast, autonom)." >&2; exit 1 ;;
esac

valid_speed() {  # valid_speed <value>: 0.1 <= value <= max_speed
  [[ "$1" =~ ^[0-9]*\.?[0-9]+$ ]] && awk -v v="$1" -v max="$max_speed" 'BEGIN { exit !(v >= 0.1 && v <= max) }'
}
ask_speed() {  # ask_speed <label> <current>: prints the chosen value ('' = keep the configured one)
  local answer
  while true; do
    read -rp "$1 speed in m/s, 0.1-${max_speed} [Enter: $2]: " answer
    if [[ -z "$answer" ]] || valid_speed "$answer"; then
      echo "$answer"
      return
    fi
    echo "  Please enter a number between 0.1 and ${max_speed}." >&2
  done
}

if [[ "$ask" == yes && -t 0 ]]; then
  if [[ "$robot" == tb3 ]]; then current_mapping=0.22 current_race=0.22; else current_mapping=0.3 current_race=0.6; fi
  mapping_speed="$(ask_speed "Mapping-lap" "$current_mapping")"
  race_speed="$(ask_speed "Race" "$current_race")"
fi
for speed in "$mapping_speed" "$race_speed"; do
  if [[ -n "$speed" ]] && ! valid_speed "$speed"; then
    echo "Speed '$speed' is not a number between 0.1 and ${max_speed} m/s for robot:=${robot}." >&2
    exit 1
  fi
done
[[ -n "$mapping_speed" ]] && launch_args+=("mapping_speed:=${mapping_speed}")
[[ -n "$race_speed" ]] && launch_args+=("race_speed:=${race_speed}")

args=("robot:=${robot}")
for arg in "${launch_args[@]}"; do
  [[ "$arg" == robot:=* ]] || args+=("$arg")
done
echo "Starting: ros2 launch robot_sim fibonacci_simulation.launch.py ${args[*]}"
exec ros2 launch robot_sim fibonacci_simulation.launch.py "${args[@]}"
