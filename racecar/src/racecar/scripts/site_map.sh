#!/usr/bin/env bash
# User-run entry point. No automatic goals, emergency resets or process killing.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/racecar/install/setup.bash"
if [[ "${1:-}" == build || "${1:-}" == nav ]]; then
  # Refuse overlapping stacks; leave existing processes entirely to the user.
  python3 - <<'PY'
from pathlib import Path
for entry in Path('/proc').glob('[0-9]*/cmdline'):
    try:
        command = entry.read_bytes().split(b'\0')
    except (OSError, ProcessLookupError):
        continue
    executable = command[0].rsplit(b'/', 1)[-1]
    if executable in (b'racecar_driver_node', b'racecar_driver_node_one', b'cartographer_node'):
        raise SystemExit('Existing car stack is running (PID ' + entry.parent.name +
                         '). Stop its launch with Ctrl+C before switching modes.')
PY
fi
case "${1:-help}" in
  build)
    shift
    exec ros2 launch racecar Run_explorer.launch.py enable_navigation:=false "$@"
    ;;
  save)
    shift
    exec ros2 run racecar save_site_map.py "$@"
    ;;
  nav)
    shift
    map_path="$HOME/maps/site_latest.yaml"
    if [[ $# -gt 0 && "$1" != *:=* ]]; then map_path="$1"; shift; fi
    exec ros2 launch racecar Run_saved_map.launch.py "map:=$map_path" "$@"
    ;;
  *)
    echo 'build: start fresh keyboard mapping (stop existing Explorer first)'
    echo 'save [name]: save /map while mapping is still running; defaults to site'
    echo 'nav [map.yaml]: open saved map + localization + navigation + RViz'
    echo 'In RViz: 2D Pose Estimate, align scan/map, then Nav2 Goal.'
    ;;
esac
