#!/usr/bin/env python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_path = get_package_share_directory("slam_omokai")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": join(pkg_path, "config", "slam_params.yaml"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        slam,
    ])
