#!/usr/bin/env bash
# Forward this viewer to the PC instead of rendering in a car-side container.
config="$HOME/racecar/rviz_remote.rviz"
if [ -f "$config" ]; then
  exec "$HOME/.local/lib/racecar-rviz-relay/rviz2" -d "$config" "$@"
fi
exec "$HOME/.local/lib/racecar-rviz-relay/rviz2" "$@"
