#!/usr/bin/env bash
set -e
if [[ "${WSL_DISTRO_NAME:-}" != "RacecarUbuntu2204" ]]; then
  echo 'This launcher runs RViz on the Windows computer in RacecarUbuntu2204 only.' >&2
  exit 1
fi
# Keep the renderer in WSLg even if an SSH/X11 DISPLAY was inherited.
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
source /mnt/d/RacecarWork/tools/rviz-renderer.conf
case "${RACECAR_RVIZ_RENDERER:-$RVIZ_RENDERER}" in
  software) export LIBGL_ALWAYS_SOFTWARE=1; export GALLIUM_DRIVER=llvmpipe ;;
  nvidia) unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER; export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ;;
  auto) unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER MESA_D3D12_DEFAULT_ADAPTER_NAME ;;
  *) echo 'Invalid RViz renderer: choose software, nvidia or auto.' >&2; exit 2 ;;
esac
echo "Local RViz renderer: ${RACECAR_RVIZ_RENDERER:-$RVIZ_RENDERER}"
source /mnt/d/RacecarWork/tools/ros2-shell.bash
export RACECAR_RVIZ_BIND_IP="$(python3 /mnt/d/RacecarWork/tools/configure-fastdds.py --print-ip)"
# Upstream geometry2 PR #990 fixes the TF callback lock inversion seen in RViz.
# Scope the library override to this local viewer, not the car or other ROS nodes.
tf_fix_lib=/opt/racecar-rviz-fixes/geometry2-0.25.23-pr990/lib
if [[ ! -r "$tf_fix_lib/libtf2.so" ]]; then
  echo 'RViz TF deadlock fix is missing; refusing to start the unfixed viewer.' >&2
  exit 1
fi
export LD_LIBRARY_PATH="$tf_fix_lib:${LD_LIBRARY_PATH:-}"
rviz_real=/opt/racecar-rviz-original/lib/rviz2/rviz2
if [[ ! -x "$rviz_real" ]]; then
  echo 'Unified RViz executable is missing; reinstall the RViz entrypoint adapter.' >&2
  exit 1
fi
export ROS_LOCALHOST_ONLY=0
if [[ "${RACECAR_RVIZ_ROUTE_CHECK:-}" == 1 ]]; then
  printf '%s\n' 'RACECAR_ROUTE=pc-local-fixed-rviz'
  printf 'RVIZ_REAL=%s\nTF_FIX=%s\n' "$rviz_real" "$tf_fix_lib/libtf2.so"
  exit 0
fi
case "${RACECAR_RVIZ_INSTANCE:-default}" in
  default) rviz_lock=/tmp/racecar-local-rviz.lock ;;
  s_curve) rviz_lock=/tmp/racecar-s-curve-rviz.lock ;;
  live_slam) rviz_lock=/tmp/racecar-live-slam-rviz.lock ;;
  *) echo 'Invalid RViz instance.' >&2; exit 2 ;;
esac
exec 9>"$rviz_lock"
if ! flock -n 9; then
  echo 'RViz instance lock is occupied; the relay must verify its configuration before reuse.' >&2
  exit 3
fi
if [ "$#" -eq 0 ]; then
  set -- -d /mnt/d/RacecarWork/tools/rviz-light.rviz
fi
args=("$@")
for ((i=0; i<${#args[@]}-1; i++)); do
  if [[ "${args[i]}" == -d || "${args[i]}" == --display-config ]]; then
    python3 /mnt/d/RacecarWork/tools/normalize-rviz-config.py "${args[i+1]}"
  fi
done
# Absolute saved binary avoids recursively entering our packaged wrappers.
exec "$rviz_real" "$@"
