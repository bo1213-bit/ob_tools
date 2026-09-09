import importlib.util
import sys
import types
import unittest
from pathlib import Path


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "three_gmsl_external_trigger.launch.py"


class LaunchConfigurationTest(unittest.TestCase):
    def test_each_camera_uses_an_isolated_official_wrapper_include(self):
        class FakeLaunchDescription:
            def __init__(self, actions):
                self.actions = actions

        class FakeGroupAction:
            def __init__(self, actions):
                self.actions = actions

        class FakeIncludeLaunchDescription:
            def __init__(self, launch_description_source, launch_arguments):
                self.launch_description_source = launch_description_source
                self.launch_arguments = dict(launch_arguments)

        class FakePythonLaunchDescriptionSource:
            def __init__(self, launch_file_path):
                self.launch_file_path = launch_file_path

        class FakeComposableNodeContainer:
            def __init__(self, **kwargs):
                self.kwargs = kwargs

        class FakeComposableNode:
            def __init__(self, **kwargs):
                self.kwargs = kwargs

        modules = {
            "ament_index_python": types.ModuleType("ament_index_python"),
            "ament_index_python.packages": types.ModuleType("ament_index_python.packages"),
            "launch": types.ModuleType("launch"),
            "launch.actions": types.ModuleType("launch.actions"),
            "launch.launch_description_sources": types.ModuleType(
                "launch.launch_description_sources"
            ),
            "launch_ros": types.ModuleType("launch_ros"),
            "launch_ros.actions": types.ModuleType("launch_ros.actions"),
            "launch_ros.descriptions": types.ModuleType("launch_ros.descriptions"),
        }
        modules["ament_index_python.packages"].get_package_share_directory = (
            lambda package_name: f"/tmp/{package_name}"
        )
        modules["launch"].LaunchDescription = FakeLaunchDescription
        modules["launch.actions"].GroupAction = FakeGroupAction
        modules["launch.actions"].IncludeLaunchDescription = FakeIncludeLaunchDescription
        modules["launch.launch_description_sources"].PythonLaunchDescriptionSource = (
            FakePythonLaunchDescriptionSource
        )
        modules["launch_ros.actions"].ComposableNodeContainer = FakeComposableNodeContainer
        modules["launch_ros.descriptions"].ComposableNode = FakeComposableNode

        original_modules = {name: sys.modules.get(name) for name in modules}
        sys.modules.update(modules)
        try:
            spec = importlib.util.spec_from_file_location(
                "three_gmsl_external_trigger", LAUNCH_FILE
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            launch_description = module.generate_launch_description()
        finally:
            for name, original in original_modules.items():
                if original is None:
                    del sys.modules[name]
                else:
                    sys.modules[name] = original

        expected_cameras = {
            "camera_305_01": {
                "serial_number": "CV3T561000B5",
                "wrapper": "gemini305.launch.py",
                "config_file_path": "/tmp/ob_ros2_timestamp_collector/config/gmsl/camera_305_01.yaml",
            },
            "camera_305_02": {
                "serial_number": "CV3T561000H0",
                "wrapper": "gemini305.launch.py",
                "config_file_path": "/tmp/ob_ros2_timestamp_collector/config/gmsl/camera_305_02.yaml",
            },
            "camera_335lg_01": {
                "serial_number": "CPBG1630011D",
                "wrapper": "gemini_330_series.launch.py",
                "config_file_path": "/tmp/ob_ros2_timestamp_collector/config/gmsl/camera_335lg_01.yaml",
            },
        }

        self.assertEqual(3, len(launch_description.actions))
        self.assertTrue(
            all(isinstance(action, FakeGroupAction) for action in launch_description.actions)
        )

        actual_cameras = {}
        for group in launch_description.actions:
            self.assertEqual(1, len(group.actions))
            include = group.actions[0]
            self.assertIsInstance(include, FakeIncludeLaunchDescription)
            arguments = include.launch_arguments
            actual_cameras[arguments["camera_name"]] = {
                "serial_number": arguments["serial_number"],
                "wrapper": Path(include.launch_description_source.launch_file_path).name,
                "config_file_path": arguments["config_file_path"].replace("\\", "/"),
            }

        self.assertEqual(expected_cameras, actual_cameras)


if __name__ == "__main__":
    unittest.main()
