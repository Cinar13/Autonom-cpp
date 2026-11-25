#!/usr/bin/env bash
# Simulation, terminal 1: Gazebo Harmonic (GUI) + RViz on the Fibonacci arena (wraps start_sim.sh).
# Then start the robot code in a second terminal: scripts/sim_robot.sh
#   scripts/sim_gazebo.sh robot:=autonom    the real robot's model (give sim_robot.sh the same robot:=)
# Any further option or launch argument is passed on unchanged; see the --help of the wrapped script.
set -euo pipefail
exec "$(dirname "${BASH_SOURCE[0]}")/start_sim.sh" "$@"
