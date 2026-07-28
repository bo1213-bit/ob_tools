#!/usr/bin/env python3
"""
Launch file for sync_analyzer_node (analysis only, no camera startup).

Use the official Orbbec launch file to start cameras first, then run this.

Usage:
  # Terminal 1: Start cameras
  ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py

  # Terminal 2: Start analysis
  ros2 launch ros2_sync_tool sync_analyzer.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    pkg_dir = get_package_share_directory("ros2_sync_tool")
    config_path = os.path.join(pkg_dir, "config", "cameras.yaml")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    analyzer_conf = config.get("analyzer", {})

    analyzer_params = {
        "camera_names": analyzer_conf.get("camera_names", []),
        "stream_types": analyzer_conf.get("stream_types", ["depth", "color"]),
        "duration_sec": analyzer_conf.get("duration_sec", 30),
        "hw_threshold_us": analyzer_conf.get("hw_threshold_us", 500),
        "csv_path": analyzer_conf.get("csv_path", ""),
    }

    analyzer_node = Node(
        package="ros2_sync_tool",
        executable="sync_analyzer_node",
        name="sync_analyzer_node",
        parameters=[analyzer_params],
        output="screen",
    )

    return LaunchDescription([analyzer_node])
