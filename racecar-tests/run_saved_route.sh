#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/racecar/install/setup.bash"
exec python3 "$HOME/racecar-tests/diagnostics/follow_saved_route.py" "$@"
