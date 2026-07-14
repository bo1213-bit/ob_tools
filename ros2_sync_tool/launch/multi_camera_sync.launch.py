#!/usr/bin/env python3
"""
Launch file for multi-camera timestamp sync analysis.

Starts N orbbec_camera_node instances (one per camera) plus the
sync_analyzer_node. Edit config/cameras.yaml to match your cameras.

Usage:
  ros2 launch ros2_sync_tool multi_camera_sync.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # Find package path
    pkg_dir = get_package_share_directory("ros2_sync_tool")
    config_path = os.path.join(pkg_dir, "config", "cameras.yaml")

    # Load configuration
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    cameras_conf = config.get("cameras", [])
    streams_conf = config.get("streams", {})
    analyzer_conf = config.get("analyzer", {})

    ld = LaunchDescription()

    # Collect camera names for the analyzer node
    camera_names = []

    # Launch one orbbec_camera_node per camera
    for cam in cameras_conf:
        camera_name = cam["name"]
        camera_names.append(camera_name)
        serial = cam.get("serial_number", "")
        sync_mode = cam.get("sync_mode", "standalone")

        # Build camera node parameters
        cam_params = {
            "camera_name": camera_name,
            "serial_number": serial,
            "sync_mode": sync_mode,
            "depth_width": streams_conf.get("depth_width", 848),
            "depth_height": streams_conf.get("depth_height", 480),
            "depth_fps": streams_conf.get("depth_fps", 30),
            "depth_format": streams_conf.get("depth_format", "Y16"),
            "color_width": streams_conf.get("color_width", 848),
            "color_height": streams_conf.get("color_height", 480),
            "color_fps": streams_conf.get("color_fps", 30),
            "color_format": streams_conf.get("color_format", "YUYV"),
            "enable_sync_host_time": True,
            "frames_per_trigger": 1,
        }

        # Primary camera enables trigger output
        if sync_mode == "primary":
            cam_params["trigger_out_enabled"] = True
            cam_params["trigger_out_delay_us"] = 0

        camera_node = Node(
            package="orbbec_camera",
            executable="orbbec_camera_node",
            name=camera_name,
            namespace=camera_name,
            parameters=[cam_params],
            output="screen",
        )
        ld.add_action(camera_node)

        ld.add_action(LogInfo(
            msg=f"Launching camera: {camera_name} (SN={serial}, sync={sync_mode})"
        ))

    # Launch the sync analyzer node
    analyzer_params = {
        "camera_names": camera_names,
        "stream_types": ["depth", "color"],
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
    ld.add_action(analyzer_node)

    ld.add_action(LogInfo(
        msg=f"Launching analyzer with cameras: {camera_names}"
    ))

    return ld