#!/usr/bin/env python3
"""
Nav2 stack in SLAM mode — no AMCL, no map server; localization and /map come
from slam_toolbox. Brings up: controller (DWB), planners (NavFn + Smac 2D),
smoother, behaviors (spin/backup/drive_on_heading/wait), BT navigator,
waypoint follower, velocity smoother, and both costmaps.

    ros2 launch slam_omokai nav2.launch.py
"""
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_path = get_package_share_directory("slam_omokai")
    nav2_share = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": join(pkg_path, "config", "nav2_params.yaml"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        nav2,
    ])
