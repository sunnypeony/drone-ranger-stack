#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    home = os.path.expanduser("~")

    # ---------------------------------------------------------
    # OAK-D
    # ---------------------------------------------------------
    # Explicitly use the Python interpreter inside oak34_ros.
    # This avoids ambiguity about which Python environment
    # actually runs the OAK-D script.
    # `source ~/oak34_ros/bin/activate`
    # ~/oak34_ros/bin/python3
    #    ↓
    # ~/natasha/oak_ros/oak_ros_rgbd_v2.py
    oak_python = os.path.join(
        home,
        "oak34_ros",
        "bin",
        "python3",
    )

    oak_script = os.path.join(
        home,
        "natasha",
        "oak_ros",
        "oak_ros_rgbd_v2.py",
    )

    oak_process = ExecuteProcess(
        cmd=[
            oak_python,
            oak_script,
        ],
        name="oak_rgbd",
        output="both",
        emulate_tty=True,
    )

    # ---------------------------------------------------------
    # YDLIDAR T-mini Plus
    # ---------------------------------------------------------
    ydlidar_share = get_package_share_directory(
        "ydlidar_ros2_driver"
    )

    ydlidar_launch_file = os.path.join(
        ydlidar_share,
        "launch",
        "ydlidar_launch.py",
    )

    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ydlidar_launch_file
        )
    )

    # ---------------------------------------------------------
    # Foxglove Bridge
    # ---------------------------------------------------------
    foxglove_share = get_package_share_directory(
        "foxglove_bridge"
    )

    foxglove_launch_file = os.path.join(
        foxglove_share,
        "launch",
        "foxglove_bridge_launch.xml",
    )

    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            foxglove_launch_file
        )
    )

    # ---------------------------------------------------------
    # Launch everything
    # ---------------------------------------------------------
    return LaunchDescription([
        LogInfo(msg="===================================="),
        LogInfo(msg=" Starting UAV Sensor Stack"),
        LogInfo(msg="   - OAK-D RGB-D"),
        LogInfo(msg="   - YDLIDAR T-mini Plus"),
        LogInfo(msg="   - Foxglove Bridge"),
        LogInfo(msg="===================================="),

        oak_process,
        ydlidar,
        foxglove,
    ])