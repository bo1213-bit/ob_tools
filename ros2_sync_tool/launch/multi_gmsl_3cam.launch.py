import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution


def generate_launch_description():
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "examples/gmsl_camera")
    config_file_dir = os.path.join(package_dir, "config")
    secondary_config = os.path.join(config_file_dir, "camera_secondary_params.yaml")

    attach_to_shared = TextSubstitution(text="true")
    container_name = "shared_orbbec_container"

    shared_container = Node(
        name=container_name,
        package="rclcpp_components",
        executable="component_container_mt",
        output="log",
    )

    # camera_01: 305g @ gmsl2-0
    cam1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_gmsl.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_01",
            "usb_port": "gmsl2-0",
            "device_num": "3",
            "sync_mode": "secondary_synced",
            "config_file_path": secondary_config,
            "attach_to_shared_component_container": attach_to_shared,
            "component_container_name": container_name,
        }.items(),
    )

    # camera_02: 305g @ gmsl2-1 (trigger output)
    cam2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_gmsl.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_02",
            "usb_port": "gmsl2-1",
            "device_num": "3",
            "sync_mode": "secondary_synced",
            "enable_gmsl_trigger": "true",
            "gmsl_trigger_fps": "3000",
            "config_file_path": secondary_config,
            "attach_to_shared_component_container": attach_to_shared,
            "component_container_name": container_name,
        }.items(),
    )

    # camera_03: 335Lg @ gmsl2-3
    cam3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_gmsl.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_03",
            "usb_port": "gmsl2-3",
            "device_num": "3",
            "sync_mode": "secondary_synced",
            "config_file_path": secondary_config,
            "attach_to_shared_component_container": attach_to_shared,
            "component_container_name": container_name,
        }.items(),
    )

    ld = LaunchDescription([
        shared_container,
        TimerAction(period=0.0, actions=[GroupAction([cam3])]),
        TimerAction(period=2.0, actions=[GroupAction([cam1])]),
        TimerAction(period=4.0, actions=[GroupAction([cam2])]),
    ])

    return ld
