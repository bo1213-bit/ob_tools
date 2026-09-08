from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("ob_ros2_timestamp_collector")
    default_config = f"{package_share}/config/collector.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the timestamp collector YAML parameter file.",
        ),
        Node(
            package="ob_ros2_timestamp_collector",
            executable="timestamp_collector_node",
            name="timestamp_collector_node",
            output="screen",
            parameters=[LaunchConfiguration("config_file")],
        ),
    ])
