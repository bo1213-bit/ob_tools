import contextlib
import csv
import importlib.util
import io
import os
import tempfile
import unittest
from pathlib import Path


ANALYZER_PATH = Path(__file__).parents[1] / "scripts" / "analyze_capture_sync.py"
CSV_HEADER = [
    "camera_name",
    "stream_type",
    "frame_index",
    "header_stamp_us",
    "receive_stamp_us",
    "frame_id",
    "width",
    "height",
    "encoding",
    "step",
    "data_size",
    "image_path",
]


def load_module():
    spec = importlib.util.spec_from_file_location("analyze_capture_sync", ANALYZER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_csv(path, rows, header=CSV_HEADER):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=header)
        writer.writeheader()
        for row in rows:
            writer.writerow({column: "" for column in header} | row)


def frame(camera, stream, index, header_stamp, receive_stamp=None):
    return {
        "camera_name": camera,
        "stream_type": stream,
        "frame_index": index,
        "header_stamp_us": header_stamp,
        "receive_stamp_us": receive_stamp if receive_stamp is not None else header_stamp,
    }


class CaptureInputTest(unittest.TestCase):
    def test_resolves_capture_directory_and_validates_required_columns(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            capture_directory = Path(temporary_directory) / "capture"
            capture_directory.mkdir()
            csv_path = capture_directory / "timestamps.csv"
            write_csv(csv_path, [frame("camera_1", "color", 0, 1_000_000)])

            self.assertEqual(csv_path, module.resolve_input_csv(capture_directory))
            self.assertEqual(csv_path, module.resolve_input_csv(csv_path))

            invalid_csv = Path(temporary_directory) / "invalid.csv"
            invalid_header = [column for column in CSV_HEADER if column != "receive_stamp_us"]
            write_csv(invalid_csv, [], header=invalid_header)
            with self.assertRaisesRegex(ValueError, "missing required column: receive_stamp_us"):
                module.load_capture(invalid_csv)

    def test_defaults_to_timestamps_csv_in_current_directory(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            capture_directory = Path(temporary_directory) / "capture"
            capture_directory.mkdir()
            csv_path = capture_directory / "timestamps.csv"
            write_csv(csv_path, [frame("camera_1", "color", 0, 1_000_000)])
            output = io.StringIO()
            previous_directory = Path.cwd()
            try:
                os.chdir(capture_directory)
                with contextlib.redirect_stdout(output):
                    result = module.main([])
            finally:
                os.chdir(previous_directory)

            self.assertEqual(0, result)
            report = output.getvalue()
            self.assertIn("Capture timestamp precision analysis", report)
            self.assertIn("Input CSV  : timestamps.csv", report)


class GroupingTest(unittest.TestCase):
    def test_groups_nearby_frames_once_and_counts_unmatched_outlier(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            csv_path = Path(temporary_directory) / "timestamps.csv"
            write_csv(
                csv_path,
                [
                    frame("camera_a", "color", 0, 1_000_000),
                    frame("camera_a", "color", 1, 1_033_333),
                    frame("camera_a", "color", 2, 1_066_666),
                    frame("camera_b", "color", 0, 1_000_400),
                    frame("camera_b", "color", 1, 1_033_733),
                    frame("camera_c", "color", 0, 1_000_900),
                    frame("camera_c", "color", 1, 1_034_233),
                ],
            )
            streams = {
                camera: records
                for (camera, stream), records in module.load_capture(csv_path).items()
                if stream == "color"
            }

            groups, unmatched = module.group_synced_frames(streams, 16_667)

            self.assertEqual(2, len(groups))
            self.assertEqual(1, unmatched)
            self.assertEqual(
                [900, 900],
                [
                    max(record.header_stamp_us for record in group.values())
                    - min(record.header_stamp_us for record in group.values())
                    for group in groups
                ],
            )


class ReportTest(unittest.TestCase):
    def test_prints_header_timestamp_precision_for_depth_color_pairs(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            csv_path = Path(temporary_directory) / "timestamps.csv"
            write_csv(
                csv_path,
                [
                    frame("camera_1", "color", 0, 1_000_300),
                    frame("camera_1", "color", 1, 1_033_833),
                    frame("camera_1", "depth", 0, 1_000_000),
                    frame("camera_1", "depth", 1, 1_033_333),
                ],
            )
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                result = module.main([str(csv_path), "--fps", "30"])

            self.assertEqual(0, result)
            report = output.getvalue()
            self.assertIn("Capture timestamp precision analysis", report)
            self.assertIn("ROS header_stamp_us", report)
            self.assertIn("========== Sensor: COLOR ==========", report)
            self.assertIn("========== Sensor: DEPTH ==========", report)
            self.assertIn("========== Intra-device checks", report)
            self.assertIn("pairs", report)
            self.assertIn("header diff: -500 ~ -300 us", report)
            self.assertIn("Abnormal: 0/2 (0.0%)", report)

    def test_rejects_invalid_fps_and_skips_single_camera_sync_check(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            csv_path = Path(temporary_directory) / "timestamps.csv"
            write_csv(csv_path, [frame("camera_1", "color", 0, 1_000_000)])
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                result = module.main([str(csv_path), "--fps", "30"])

            self.assertEqual(0, result)
            self.assertIn("Fewer than 2 cameras, cannot evaluate sync, skip", output.getvalue())

            error_output = io.StringIO()
            with contextlib.redirect_stderr(error_output):
                result = module.main([str(csv_path), "--fps", "0"])

            self.assertEqual(1, result)
            self.assertIn("--fps must be greater than 0", error_output.getvalue())


if __name__ == "__main__":
    unittest.main()
