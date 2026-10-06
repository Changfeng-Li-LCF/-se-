#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/racecar/install/setup.bash"
cd "$HOME/racecar-tests/s-curve-test"
exec python3 run_session.py "$@"
