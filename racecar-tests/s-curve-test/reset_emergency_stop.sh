#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/racecar/install/setup.bash"
exec ros2 service call /racecar_driver/reset_emergency_stop std_srvs/srv/Trigger '{}'
