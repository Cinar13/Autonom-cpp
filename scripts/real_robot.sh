#!/usr/bin/env bash
# Real robot, on the Raspberry Pi 5: hardware drivers + the robot code (wraps start_robot.sh).
#   scripts/real_robot.sh --imu --controller mppi
# Never starts Gazebo: simulation options are refused.
# Any further option or launch argument is passed on unchanged; see the --help of the wrapped script.
set -euo pipefail
for arg in "$@"; do
  case "$arg" in
    --sim | --view | stack:=* | robot:=* | world:=* | gui:=*)
      echo "$arg is a simulation option; real_*.sh never start Gazebo. Use sim_gazebo.sh / sim_robot.sh." >&2
      exit 1 ;;
  esac
done
if pgrep -f '(^|/)gz sim( |$)' >/dev/null; then
  echo "A Gazebo simulation is running on this machine; stop it first (its topics would mix with the robot's)." >&2
  exit 1
fi
exec "$(dirname "${BASH_SOURCE[0]}")/start_robot.sh" "$@"
