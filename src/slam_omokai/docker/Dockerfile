# syntax=docker/dockerfile:1
# SLAM_OMOKAI — Gazebo Harmonic + slam_toolbox + Nav2 (ROS 2 Humble)
#
# Build (from the slam_omokai package root):
#   docker build -t slam_omokai -f docker/Dockerfile .
# or just: docker compose build     (see docker-compose.yml)

FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# ── OSRF packages repo ────────────────────────────────────────────────────
# gz-harmonic and the Harmonic variant of ros_gz (ros-humble-ros-gzharmonic)
# are NOT in the regular ROS repo for Humble — they live in the OSRF
# gazebo-stable repo. The default ros-humble-ros-gz targets Fortress and is
# incompatible with this project's gz-sim-8 plugin names.
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# ── Runtime stack ─────────────────────────────────────────────────────────
#   gz-harmonic                     gz sim 8 (physics, gpu_lidar, GUI)
#   ros-humble-ros-gzharmonic       ros_gz_sim + ros_gz_bridge for Harmonic
#   navigation2 / nav2-bringup      the whole Nav2 stack (binary, §1 of NAV2_ARCHITECTURE.md)
#   slam-toolbox                    online async SLAM
#   xacro / robot-state-publisher   URDF processing + TF
#   rviz2                           visualization (GUI profile)
#   mesa GL/EGL                     rendering for gpu_lidar (EGL headless too)
RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-rviz2 \
        python3-colcon-common-extensions \
        libgl1-mesa-dri libglx-mesa0 libegl-mesa0 libegl1 \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace ─────────────────────────────────────────────────────────────
WORKDIR /ws
COPY . src/slam_omokai/
# Plain (copy) install — NOT --symlink-install: build/ is deleted afterwards
# to slim the image, and symlink-install would leave install/ pointing into it.
RUN . /opt/ros/humble/setup.sh \
    && colcon build \
    && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Fast DDS: UDP only, no shared memory. The container runs as root, so its
# /dev/shm segments are unreadable by the host user — with SHM enabled,
# host<->container discovery works but data silently never flows.
COPY docker/fastdds_udp_only.xml /fastdds_udp_only.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp_only.xml

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "slam_omokai", "bringup.launch.py"]
