#!/usr/bin/env python3
"""
    ros2 launch slam_omokai bringup.launch.py rviz:=false
    ros2 launch slam_omokai bringup.launch.py headless:=true   # no Gazebo GUI


IMPORTANT: Nav2 only becomes active ~25 s after launch (sim must be up first).
Goals clicked before the "NAV2 IS ACTIVE" banner appears in this terminal are
silently dropped — wait for the banner, then click. Every accepted goal logs
"Begin navigating from ..." here; if you click and that line never shows up,
the goal never reached Nav2.
"""
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("slam_omokai")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    headless = LaunchConfiguration("headless", default="false")
    rviz_enabled = LaunchConfiguration("rviz", default="true")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_path, "launch", "sim.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "headless": headless,
        }.items(),
    )

    # Delay SLAM + Nav2 a little so the sim, bridge and /clock are up first.
    slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(pkg_path, "launch", "slam.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(pkg_path, "launch", "nav2.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    # Goals sent before bt_navigator is active are dropped without any feedback,
    # so tell the user exactly when the stack is ready to accept them.
    nav2_ready_banner = TimerAction(
        period=9.0,
        actions=[
            ExecuteProcess(
                name="nav2_ready",
                cmd=[
                    "bash", "-c",
                    "until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active; "
                    "do sleep 2; done; "
                    "printf '\\n============================================================\\n'; "
                    "printf ' NAV2 IS ACTIVE — set goals in RViz with the Nav2 Goal tool\\n'; "
                    "printf '============================================================\\n\\n'",
                ],
                output="screen",
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", join(pkg_path, "rviz", "slam_nav.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz_enabled),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        sim,
        slam,
        nav2,
        nav2_ready_banner,
        rviz,
    ])
