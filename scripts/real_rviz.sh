#!/usr/bin/env bash
# Real robot, main PC: RViz watching the robot over CycloneDDS, no simulation (wraps start_sim.sh --view).
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
exec "$(dirname "${BASH_SOURCE[0]}")/start_sim.sh" --view "$@"
