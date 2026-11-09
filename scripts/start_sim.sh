#!/usr/bin/env bash
# Start the race simulation: Gazebo Harmonic (GUI) + RViz + the full autonomy stack on the Fibonacci arena.
#
#   scripts/start_sim.sh                       asks for the speeds and the Nav2 mode, then whether to change
#                                              accelerations / turn speed
#   scripts/start_sim.sh --speed 0.8           race laps at 0.8 m/s, no questions
#   scripts/start_sim.sh --speed 1.2 --controller mppi --planner thetastar
#   scripts/start_sim.sh --mapping-speed 0.25 --speed 0.5 --race-accel 2.0 --turn-speed 4.0
#   scripts/start_sim.sh --defaults            configured values, no questions
#   scripts/start_sim.sh robot:=autonom        the real robot's model; any fibonacci_simulation argument can be
#   scripts/start_sim.sh yaw:=3.1416           added, e.g. race the other way round
#
# Options (empty / Enter keeps the configured value):
#   --mapping-speed, --speed (--race-speed)    0.1-1.5 m/s (robot:=autonom up to 1.46, its motors' 430 rpm)
#   --mapping-accel, --race-accel              0.1-3.0 m/s^2 (configured 0.5 / 1.5)
#   --turn-speed                               0.5-5.0 rad/s (configured 3.0; tb3 2.0)
#   --controller                               rpp (default) | mppi (RPP maps, MPPI races) | graceful (sim only)
#   --planner                                  navfn (default) | smac | thetastar
# Nav2 modes measured in docs/SESSION_2026-11-09.md: MPPI races fastest at 0.6 m/s, MPPI and RPP + Theta* at 1.2 m/s.
# Above 0.6 m/s only robot:=tb3_fast is tested (up to 1.25 m/s). Needs a built workspace (scripts/install.sh). Ctrl+C stops everything.
set -euo pipefail

declare -A value=([mapping_speed]="" [race_speed]="" [mapping_accel]="" [race_accel]="" [turn_speed]="")
controller=""
planner=""
ask=yes
launch_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --speed | --race-speed) value[race_speed]="$2"; ask=no; shift 2 ;;
    --mapping-speed) value[mapping_speed]="$2"; ask=no; shift 2 ;;
    --mapping-accel) value[mapping_accel]="$2"; ask=no; shift 2 ;;
    --race-accel) value[race_accel]="$2"; ask=no; shift 2 ;;
    --turn-speed) value[turn_speed]="$2"; ask=no; shift 2 ;;
    --controller) controller="$2"; shift 2 ;;
    --planner) planner="$2"; shift 2 ;;
    --defaults) ask=no; shift ;;
    -h | --help) sed -n '2,20p' "$0"; exit 0 ;;
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

if pgrep -f '(^|/)gz sim( |$)' >/dev/null; then
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
  tb3) max_speed=1.5 default_mapping=0.22 default_race=0.22 default_turn=2.0 ;;
  tb3_fast) max_speed=1.5 default_mapping=0.3 default_race=0.6 default_turn=3.0 ;;
  autonom) max_speed=1.46 default_mapping=0.3 default_race=0.6 default_turn=3.0 ;;
  *) echo "Unknown robot '$robot' (tb3, tb3_fast, autonom)." >&2; exit 1 ;;
esac
# name -> "low high unit label default"
declare -A spec=(
  [mapping_speed]="0.1 ${max_speed} m/s Mapping-lap_speed ${default_mapping}"
  [race_speed]="0.1 ${max_speed} m/s Race_speed ${default_race}"
  [mapping_accel]="0.1 3.0 m/s^2 Mapping-lap_acceleration 0.5"
  [race_accel]="0.1 3.0 m/s^2 Race_acceleration 1.5"
  [turn_speed]="0.5 5.0 rad/s Turn_speed ${default_turn}"
)

in_range() {  # in_range <value> <low> <high>
  [[ "$1" =~ ^[0-9]*\.?[0-9]+$ ]] && awk -v v="$1" -v lo="$2" -v hi="$3" 'BEGIN { exit !(v >= lo && v <= hi) }'
}
ask_value() {  # ask_value <name>: prints the answer ('' = keep the configured value)
  local low high unit label default answer
  read -r low high unit label default <<< "${spec[$1]}"
  while true; do
    read -rp "${label//_/ } in ${unit}, ${low}-${high} [Enter: ${default}]: " answer
    if [[ -z "$answer" ]] || in_range "$answer" "$low" "$high"; then
      echo "$answer"
      return
    fi
    echo "  Please enter a number between ${low} and ${high}." >&2
  done
}

for arg in "${launch_args[@]}"; do  # controller:= / planner:= given the launch way count too
  [[ "$arg" == controller:=* ]] && controller="${arg#controller:=}"
  [[ "$arg" == planner:=* ]] && planner="${arg#planner:=}"
done
ask_choice() {  # ask_choice <label> <default> <choices...>: prints the answer
  local label="$1" default="$2" answer options
  shift 2
  options="$(printf ' | %s' "$@")"
  while true; do
    read -rp "${label} (${options:3}) [Enter: ${default}]: " answer
    answer="${answer:-$default}"
    for choice in "$@"; do
      [[ "$answer" == "$choice" ]] && { echo "$answer"; return; }
    done
    echo "  Please enter one of: $*" >&2
  done
}

if [[ "$ask" == yes && -t 0 ]]; then
  value[mapping_speed]="$(ask_value mapping_speed)"
  value[race_speed]="$(ask_value race_speed)"
  [[ -z "$controller" ]] && controller="$(ask_choice "Path follower (mppi: RPP maps, MPPI races)" rpp rpp mppi graceful)"
  [[ -z "$planner" ]] && planner="$(ask_choice "Global planner" navfn navfn smac thetastar)"
  read -rp "Change the accelerations and turn speed too? [y/N] " more
  if [[ "$more" =~ ^[Yy] ]]; then
    value[mapping_accel]="$(ask_value mapping_accel)"
    value[race_accel]="$(ask_value race_accel)"
    value[turn_speed]="$(ask_value turn_speed)"
  fi
fi
for name in mapping_speed race_speed mapping_accel race_accel turn_speed; do
  [[ -z "${value[$name]}" ]] && continue
  read -r low high unit _ _ <<< "${spec[$name]}"
  if ! in_range "${value[$name]}" "$low" "$high"; then
    echo "${name//_/ } '${value[$name]}' is not a number between ${low} and ${high} ${unit} for robot:=${robot}." >&2
    exit 1
  fi
  launch_args+=("${name}:=${value[$name]}")
done

controller="${controller:-rpp}"
planner="${planner:-navfn}"
case "$controller" in rpp | mppi | graceful) ;; *) echo "Unknown controller '$controller' (rpp, mppi, graceful)." >&2; exit 1 ;; esac
case "$planner" in navfn | smac | thetastar) ;; *) echo "Unknown planner '$planner' (navfn, smac, thetastar)." >&2; exit 1 ;; esac

args=("robot:=${robot}" "controller:=${controller}" "planner:=${planner}")
for arg in "${launch_args[@]}"; do
  [[ "$arg" == robot:=* || "$arg" == controller:=* || "$arg" == planner:=* ]] || args+=("$arg")
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
ros2 launch robot_sim fibonacci_simulation.launch.py "${args[@]}"
