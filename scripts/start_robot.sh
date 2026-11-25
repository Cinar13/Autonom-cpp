#!/usr/bin/env bash
# Start the robot code: EKF + slam_toolbox + Nav2 + loop patrol + start gate. No GUI application runs here.
# On the robot (Raspberry Pi 5) with the hardware drivers; with --sim against the simulation of scripts/start_sim.sh
# (start that first, in another terminal). Watch the real robot from the base station with: scripts/start_sim.sh --view
#
#   scripts/start_robot.sh                                   wheels + rf2o, RPP + NavFn
#   scripts/start_robot.sh --controller mppi                 RPP maps, MPPI races (check its CPU on the Pi)
#   scripts/start_robot.sh --planner thetastar --imu         Theta* planner, BNO055 fused in the EKF
#   scripts/start_robot.sh --map-only                        no loop patrol (map / teleop only)
#   scripts/start_robot.sh lidar_serial_port:=/dev/ttyUSB1   any bringup.launch.py argument can be added
#
#   scripts/start_robot.sh --sim                             asks for the speeds and the Nav2 mode, then whether to
#                                                            change accelerations / turn speed
#   scripts/start_robot.sh --sim --speed 0.8                 race laps at 0.8 m/s, no questions
#   scripts/start_robot.sh --sim --speed 1.2 --controller mppi --planner thetastar
#   scripts/start_robot.sh --sim --mapping-speed 0.25 --speed 0.5 --race-accel 2.0 --turn-speed 4.0
#   scripts/start_robot.sh --sim --defaults                  configured values, no questions
#   scripts/start_robot.sh --sim robot:=autonom              must match the robot:= given to start_sim.sh
#
# Options:
#   --controller   rpp (default) | mppi (RPP maps, MPPI races) | graceful (--sim only)
#   --planner      navfn (default) | smac | thetastar
#   --map-only     do not run the loop patrol (autonomous_loop:=false)
#   --imu          robot: start the BNO055 driver and fuse /imu/data (the simulation fuses its IMU by default)
#   --no-rf2o      wheel odometry only, no rf2o laser odometry (use_rf2o:=false)
#   --sim          against the running simulation, local or on the LAN (use_sim_time); robot:= tb3 | tb3_fast (default) | autonom
#   --sim only (empty / Enter keeps the configured value):
#     --mapping-speed, --speed (--race-speed)  0.1-1.5 m/s (robot:=autonom up to 1.46, its motors' 430 rpm)
#     --mapping-accel, --race-accel            0.1-3.0 m/s^2 (configured 0.5 / 1.5)
#     --turn-speed                             0.5-5.0 rad/s (configured 3.0; tb3 2.0)
# Nav2 modes measured in docs/SESSION_2026-11-09.md: MPPI races fastest at 0.6 m/s, MPPI and RPP + Theta* at 1.2 m/s.
# Above 0.6 m/s only robot:=tb3_fast is tested (up to 1.25 m/s).
# On the robot nothing moves until the race starts: publish std_msgs/Empty on /race/start_signal; the start gate then
# holds /cmd_vel for the 5 s start delay (docs/06_competition_rules.md). The simulation sends the start signal itself.
# Needs a built workspace. Ctrl+C stops everything.
set -euo pipefail

declare -A value=([mapping_speed]="" [race_speed]="" [mapping_accel]="" [race_accel]="" [turn_speed]="")
controller=""
planner=""
sim=no
ask=yes
launch_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim) sim=yes; shift ;;
    --controller) controller="$2"; shift 2 ;;
    --planner) planner="$2"; shift 2 ;;
    --imu) launch_args+=("use_imu:=true"); shift ;;
    --no-rf2o) launch_args+=("use_rf2o:=false"); shift ;;
    --map-only) launch_args+=("autonomous_loop:=false"); shift ;;
    --speed | --race-speed) value[race_speed]="$2"; ask=no; shift 2 ;;
    --mapping-speed) value[mapping_speed]="$2"; ask=no; shift 2 ;;
    --mapping-accel) value[mapping_accel]="$2"; ask=no; shift 2 ;;
    --race-accel) value[race_accel]="$2"; ask=no; shift 2 ;;
    --turn-speed) value[turn_speed]="$2"; ask=no; shift 2 ;;
    --defaults) ask=no; shift ;;
    -h | --help) sed -n '2,35p' "$0"; exit 0 ;;
    controller:=*) controller="${1#controller:=}"; shift ;;
    planner:=*) planner="${1#planner:=}"; shift ;;
    *) launch_args+=("$1"); shift ;;
  esac
done

if [[ "$sim" == no ]]; then
  for name in "${!value[@]}"; do
    if [[ -n "${value[$name]}" ]]; then
      echo "--${name//_/-} is only available with --sim (the robot runs its configured loop_patrol.yaml)." >&2
      exit 1
    fi
  done
  for arg in "${launch_args[@]}"; do
    if [[ "$arg" == robot:=* ]]; then
      echo "robot:= is only available with --sim." >&2
      exit 1
    fi
  done
fi
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

if [[ "$sim" == no ]]; then
  if pgrep -f '(^|/)gz sim( |$)' >/dev/null; then
    echo "A Gazebo simulation is running on this machine; its topics would mix with the robot's." >&2
    echo "Stop it first, or run the robot code against it: scripts/start_robot.sh --sim" >&2
    exit 1
  fi
  # Same DDS setup as the base station (README, "CycloneDDS"); without it RViz there may not see the robot.
  if [[ "${RMW_IMPLEMENTATION:-}" != rmw_cyclonedds_cpp ]]; then
    echo "Warning: RMW_IMPLEMENTATION is not rmw_cyclonedds_cpp (see README, CycloneDDS)." >&2
  fi
  controller="${controller:-rpp}"
  planner="${planner:-navfn}"
  case "$controller" in rpp | mppi) ;; *) echo "Unknown controller '$controller' (rpp, mppi)." >&2; exit 1 ;; esac
  case "$planner" in navfn | smac | thetastar) ;; *) echo "Unknown planner '$planner' (navfn, smac, thetastar)." >&2; exit 1 ;; esac

  # A stale ROS 2 CLI daemon can hide topics from `ros2 topic` commands in other terminals.
  ros2 daemon stop >/dev/null 2>&1 || true

  args=("controller:=${controller}" "planner:=${planner}" "${launch_args[@]}")
  echo "Starting: ros2 launch autonom_bringup bringup.launch.py ${args[*]}"
  exec ros2 launch autonom_bringup bringup.launch.py "${args[@]}"
fi

# --sim: the robot code against the simulation of scripts/start_sim.sh, on this machine or another one on the LAN.
# A stale ROS 2 CLI daemon can hide topics from `ros2 topic` commands in other terminals.
ros2 daemon stop >/dev/null 2>&1 || true

# Gazebo may run on another machine, so look for the simulation's /clock over DDS instead of a local gz process.
if ! pgrep -f '(^|/)gz sim( |$)' >/dev/null; then
  echo "No local Gazebo; waiting up to 20 s for the simulation's /clock on the network ..."
  if ! timeout 20 ros2 topic echo --once --qos-reliability best_effort /clock rosgraph_msgs/msg/Clock \
      >/dev/null 2>&1; then
    echo "No simulation found: /clock is not visible from this machine." >&2
    echo "Start it first (scripts/start_sim.sh), here or on another machine. If it runs on another machine:" >&2
    echo "  - same ROS_DOMAIN_ID on both (here: ${ROS_DOMAIN_ID:-0}), ROS_AUTOMATIC_DISCOVERY_RANGE not LOCALHOST" >&2
    echo "    (here: ${ROS_AUTOMATIC_DISCOVERY_RANGE:-unset}), same RMW_IMPLEMENTATION (here: ${RMW_IMPLEMENTATION:-unset})" >&2
    echo "  - CYCLONEDDS_URI either set on BOTH machines or on neither (here: ${CYCLONEDDS_URI:-unset}); with the" >&2
    echo "    repo's cyclonedds.xml (unicast only) AUTONOM_RPI_IP / AUTONOM_WSL_IP must hold the two machines' LAN IPs" >&2
    echo "  - firewall open for UDP 7400-7700 (e.g. sudo ufw allow from <other machine's IP>)" >&2
    exit 1
  fi
  echo "Simulation found on the network."
fi

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

args=("stack:=robot" "robot:=${robot}" "controller:=${controller}" "planner:=${planner}")
for arg in "${launch_args[@]}"; do
  [[ "$arg" == robot:=* || "$arg" == stack:=* ]] || args+=("$arg")
done
echo "Starting: ros2 launch robot_sim fibonacci_simulation.launch.py ${args[*]}"
exec ros2 launch robot_sim fibonacci_simulation.launch.py "${args[@]}"
