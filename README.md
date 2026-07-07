# SLAM_OMOKAI

ROS 2 Humble workspace: a differential-drive robot in Gazebo Harmonic that maps
its world live with **slam_toolbox** and navigates autonomously with **Nav2** —
no pre-built map needed.

**The package (all source, docs and Docker setup) lives in
[`src/slam_omokai/`](src/slam_omokai/):**

- [`src/slam_omokai/README.md`](src/slam_omokai/README.md) — usage, goal
  sending, troubleshooting
- [`src/slam_omokai/NAV2_ARCHITECTURE.md`](src/slam_omokai/NAV2_ARCHITECTURE.md)
  — full top-to-bottom system/Nav2 deep dive
- [`src/slam_omokai/docker/README.md`](src/slam_omokai/docker/README.md) — run
  everything in Docker, no ROS install needed

## Quick start (native)

```bash
cd SLAM_OMOKAI
colcon build
source install/setup.bash
ros2 launch slam_omokai bringup.launch.py    # sim + SLAM + Nav2 + RViz
```

## Quick start (Docker)

```bash
cd src/slam_omokai
./docker/run.sh build
./docker/run.sh gui        # or: headless
```

Wait for the `NAV2 IS ACTIVE` banner (~25 s), then set goals in RViz with the
**Nav2 Goal** tool.
