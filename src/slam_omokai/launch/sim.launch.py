#!/usr/bin/env python3
"""
Gazebo Harmonic (gz sim 8) + robot spawn + ros_gz bridge + robot_state_publisher.

    ros2 launch slam_omokai sim.launch.py
    ros2 launch slam_omokai sim.launch.py headless:=true   # server only, no GUI
"""
import tempfile
from os.path import join

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_path = get_package_share_directory("slam_omokai")
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    headless = LaunchConfiguration("headless", default="false")
    world_file = LaunchConfiguration(
        "world_file", default=join(pkg_path, "worlds", "omokai_world.sdf")
    )

    # -r: run immediately; -s: server only (headless)
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"]),
        }.items(),
        condition=UnlessCondition(headless),
    )
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r -s --headless-rendering'"]),
        }.items(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(
                    Command(["xacro ", join(pkg_path, "urdf", "omokai_bot.xacro")]),
                    value_type=str,
                ),
            }
        ],
    )

    # Spawn from a generated URDF file: spawning via "-topic /robot_description"
    # is silently dropped by this gz-sim/ros_gz combination (the create service
    # replies OK but no entity ever appears); spawning from a file works.
    robot_urdf = xacro.process_file(
        join(pkg_path, "urdf", "omokai_bot.xacro")
    ).toxml()
    urdf_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="omokai_bot_", suffix=".urdf", delete=False
    )
    urdf_file.write(robot_urdf)
    urdf_file.close()

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", urdf_file.name,
            "-name", "omokai_bot",
            "-allow_renaming", "true",
            "-x", "-5.0",
            "-y", "-5.0",
            "-z", "0.2",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/world/omokai_world/model/omokai_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[
            ("/world/omokai_world/model/omokai_bot/joint_state", "/joint_states"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        # Pin gz-transport to loopback: with multiple interfaces present it can
        # bind to docker0 (172.17.0.1, often DOWN), and the sim<->bridge data
        # channels then silently die mid-run — /clock, /scan and /odom freeze,
        # Nav2 stops accepting goals while every process still looks alive.
        SetEnvironmentVariable(name="GZ_IP", value="127.0.0.1"),
        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH", value=join(pkg_path, "worlds")
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("world_file", default_value=world_file),
        gz_sim_gui,
        gz_sim_server,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
