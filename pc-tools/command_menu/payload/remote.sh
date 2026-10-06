#!/usr/bin/env bash
source /opt/ros/humble/setup.bash || exit
source "$HOME/racecar/install/setup.bash" || exit
exec python3 "$(dirname "$0")/dispatch.py" "$@"
