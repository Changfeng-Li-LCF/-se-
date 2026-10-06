#!/usr/bin/env bash
set -e
# The shared runner reads the current reference_local.json and opens RViz.
exec bash "$HOME/racecar-tests/s-curve-test/run_s_curve.sh" "$@"
