#!/bin/bash
# Helper for machines without the docker compose plugin.
#   ./docker/run.sh build      build the image
#   ./docker/run.sh gui        sim + SLAM + Nav2 + Gazebo GUI + RViz
#   ./docker/run.sh headless   server only, no GUI
#   ./docker/run.sh shell      bash inside the image
set -e
cd "$(dirname "$0")/.."

IMAGE=slam_omokai:humble

COMMON=(
  --rm -it
  --network host --ipc host
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
)

# ── GPU selection ──────────────────────────────────────────────────────────
# The image only ships Mesa drivers, so pass in render nodes Mesa can drive
# (Intel/AMD) and SKIP NVIDIA nodes — if EGL enumerates an NVIDIA node
# without the NVIDIA userspace libs, gz sim's ogre2 engine segfaults on
# startup. No usable node → fall back to CPU rendering (llvmpipe): slower
# but works everywhere. NVIDIA users with nvidia-container-toolkit can
# instead run with GPUS=nvidia to use --gpus all.
if [ "${GPUS:-auto}" = "nvidia" ]; then
  COMMON+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
else
  found_gpu=0
  for node in /dev/dri/renderD*; do
    [ -e "$node" ] || continue
    drv=$(basename "$(readlink -f "/sys/class/drm/$(basename "$node")/device/driver")" 2>/dev/null || true)
    if [ -n "$drv" ] && [ "$drv" != "nvidia" ]; then
      COMMON+=(--device "$node")
      found_gpu=1
    fi
  done
  if [ "$found_gpu" = 0 ]; then
    echo "[run.sh] no Mesa-compatible GPU render node found — using CPU rendering" >&2
    COMMON+=(-e LIBGL_ALWAYS_SOFTWARE=1)
  fi
fi

X11=(
  -e DISPLAY="${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
)

case "${1:-gui}" in
  build)
    docker build -t "$IMAGE" -f docker/Dockerfile .
    ;;
  gui)
    xhost +local: >/dev/null 2>&1 || true
    docker run "${COMMON[@]}" "${X11[@]}" "$IMAGE"
    ;;
  headless)
    docker run "${COMMON[@]}" "$IMAGE" \
      ros2 launch slam_omokai bringup.launch.py headless:=true rviz:=false
    ;;
  shell)
    xhost +local: >/dev/null 2>&1 || true
    docker run "${COMMON[@]}" "${X11[@]}" "$IMAGE" bash
    ;;
  *)
    echo "usage: $0 {build|gui|headless|shell}" >&2
    exit 1
    ;;
esac
