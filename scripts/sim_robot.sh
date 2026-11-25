#!/usr/bin/env bash
# Simulation, terminal 2: the robot code against the simulation of scripts/sim_gazebo.sh
# (wraps start_robot.sh --sim). Asks for the speeds and the Nav2 mode.
#   scripts/sim_robot.sh --speed 0.8 --controller mppi
#   scripts/sim_robot.sh robot:=autonom     must match the robot:= given to sim_gazebo.sh
# Any further option or launch argument is passed on unchanged; see the --help of the wrapped script.
set -euo pipefail
exec "$(dirname "${BASH_SOURCE[0]}")/start_robot.sh" --sim "$@"
