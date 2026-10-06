#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/racecar/install/setup.bash"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$ROOT/run_session.py" --saved-route "$ROOT/marked_route.json" "$@"
