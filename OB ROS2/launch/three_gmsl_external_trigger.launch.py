import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


CAMERAS = (
    ("gemini305.launch.py", "camera_305_01", "CV3T561000B5", "camera_305_01.yaml"),
    ("gemini305.launch.py", "camera_305_02", "CV3T561000H0", "camera_305_02.yaml"),
    (
        "gemini_330_series.launch.py",
        "camera_335lg_01",
        "CPBG1630011D",
        "camera_335lg_01.yaml",
    ),
)


def generate_launch_description():
    package_share = get_package_share_directory("ob_ros2_timestamp_collector")
    orbbec_share = get_package_share_directory("orbbec_camera")
    actions = []

    for wrapper_name, camera_name, serial_number, config_name in CAMERAS:
        config_path = os.path.join(package_share, "config", "gmsl", config_name)
        wrapper_path = os.path.join(orbbec_share, "launch", wrapper_name)
        actions.append(
            GroupAction(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(wrapper_path),
                        launch_arguments={
                            "camera_name": camera_name,
                            "serial_number": serial_number,
                            "config_file_path": config_path,
                        }.items(),
                    )
                ]
            )
        )

    return LaunchDescription(actions)
