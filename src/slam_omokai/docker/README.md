# Running SLAM_OMOKAI in Docker

Everything (Gazebo Harmonic, slam_toolbox, Nav2, RViz) baked into one image on
`ros:humble-ros-base`. The image installs the **Harmonic** variant of ros_gz
(`ros-humble-ros-gzharmonic` from the OSRF repo) — the default
`ros-humble-ros-gz` targets Fortress and cannot load this robot's
`gz-sim-*-system` plugins.

## Quick start

```bash
cd ~/SLAM_OMOKAI/src/slam_omokai

docker compose build                      # ~10 min first time (downloads gz-harmonic + Nav2)

xhost +local:                             # let the container open windows on your X server
docker compose --profile gui up           # sim + SLAM + Nav2 + Gazebo GUI + RViz
```

Wait for the `NAV2 IS ACTIVE` banner (~25 s), then click goals in RViz as usual.

## Headless (no GUI, e.g. on a server / CI)

```bash
docker compose --profile headless up
```

Gazebo runs with `--headless-rendering` (EGL — the gpu_lidar still needs a GL
context, provided by `/dev/dri` or the software fallback below). Send goals
from the host — host networking means host ROS 2 tools see the container's
topics directly:

```bash
ros2 topic pub -w 1 --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
```

Or run RViz on the host against the containerized stack:

```bash
rviz2 -d ~/SLAM_OMOKAI/src/slam_omokai/rviz/slam_nav.rviz
```

## Plain docker (no compose)

```bash
docker build -t slam_omokai -f docker/Dockerfile .

docker run -it --rm \
  --network host --ipc host \
  --device /dev/dri/renderD128 \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  slam_omokai
```

(`renderD128` = the Intel iGPU on this machine — see the GPU table below for
other setups.)

Append any other command to override the default bringup, e.g. a shell:

```bash
docker run -it --rm --network host --ipc host slam_omokai bash
```

## GPU / rendering options

`run.sh` handles this automatically: it passes in every **Mesa-drivable**
(Intel/AMD) render node, skips NVIDIA nodes, and falls back to CPU rendering
if none exist. If you use compose or plain docker instead:

| Situation | What to do |
|---|---|
| Intel/AMD GPU | Map that GPU's render node, e.g. `--device /dev/dri/renderD128` (find it: `ls -l /dev/dri/by-path` + `lspci \| grep -i VGA`). |
| Hybrid Intel+NVIDIA laptop | **Map only the Intel node.** Mapping all of `/dev/dri` lets EGL pick the NVIDIA node, which the image's Mesa can't drive — gz sim's ogre2 engine segfaults at startup (verified on this machine). |
| NVIDIA GPU (dGPU only) | Install `nvidia-container-toolkit`, use `--gpus all` / uncomment `gpus: all` in compose, don't map `/dev/dri`. Or with run.sh: `GPUS=nvidia ./docker/run.sh gui`. |
| No GPU at all | `-e LIBGL_ALWAYS_SOFTWARE=1`, no devices — llvmpipe CPU rendering. Slower (the 360-ray lidar at 10 Hz is the load) but verified working. |

## Why host networking?

Two reasons:

1. **DDS discovery** — with `network_mode: host` + `ipc: host`, ROS 2 nodes in
   the container and on the host discover each other automatically; no
   discovery-server or domain bridging needed.
2. **gz-transport** — the launch already pins `GZ_IP=127.0.0.1`
   (see `sim.launch.py`), which behaves identically inside the container.

If you want full isolation instead, switch to the default bridge network — the
stack is self-contained in one container so it still works, you just lose
host-side `ros2`/RViz access.

## Files

```
docker/Dockerfile        image: OSRF repo + gz-harmonic + ros-gzharmonic + Nav2
                         + slam_toolbox, then colcon-builds this package into /ws
docker/entrypoint.sh     sources /opt/ros/humble + /ws/install, execs the command
docker-compose.yml       gui / headless profiles, X11 + /dev/dri wiring
.dockerignore            keeps build/, install/, log/ out of the image
```
