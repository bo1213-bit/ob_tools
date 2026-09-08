#!/usr/bin/env python3
"""Print offline synchronization precision from collector timestamps.csv files."""

import argparse
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path
from typing import NamedTuple


REQUIRED_COLUMNS = {
    "camera_name",
    "stream_type",
    "frame_index",
    "header_stamp_us",
    "receive_stamp_us",
}
DEFAULT_FPS = 30.0
DEFAULT_THRESHOLD_US = 2000.0


class FrameRecord(NamedTuple):
    camera_name: str
    stream_type: str
    frame_index: int
    header_stamp_us: int
    receive_stamp_us: int


def resolve_input_csv(path: Path) -> Path:
    path = Path(path)
    csv_path = path / "timestamps.csv" if path.is_dir() else path
    if not csv_path.is_file():
        raise ValueError(f"input must be a timestamps.csv file or capture directory: {path}")
    return csv_path


def load_capture(csv_path: Path):
    with Path(csv_path).open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or ())
        for column in sorted(REQUIRED_COLUMNS - columns):
            raise ValueError(f"missing required column: {column}")

        streams = defaultdict(list)
        for row_number, row in enumerate(reader, start=2):
            camera_name = row["camera_name"].strip()
            stream_type = row["stream_type"].strip().lower()
            if not camera_name:
                raise ValueError(f"row {row_number}: camera_name is empty")
            if stream_type not in {"color", "depth"}:
                raise ValueError(f"row {row_number}: unsupported stream_type {stream_type!r}")
            try:
                record = FrameRecord(
                    camera_name,
                    stream_type,
                    int(row["frame_index"]),
                    int(row["header_stamp_us"]),
                    int(row["receive_stamp_us"]),
                )
            except ValueError as error:
                raise ValueError(f"row {row_number}: invalid integer field") from error
            streams[(camera_name, stream_type)].append(record)

    for records in streams.values():
        records.sort(key=lambda record: record.header_stamp_us)
    return dict(streams)


def stream_fps(records):
    gaps = [
        later.receive_stamp_us - earlier.receive_stamp_us
        for earlier, later in zip(records, records[1:])
        if 0 < later.receive_stamp_us - earlier.receive_stamp_us < 1_000_000
    ]
    if not gaps:
        return None
    return 1_000_000.0 / statistics.median(gaps)


def infer_fps(streams):
    values = [stream_fps(records) for records in streams.values()]
    values = [value for value in values if value is not None]
    return statistics.median(values) if values else None


def nearest_index(records, start, target_us, half_gap_us):
    best_index = -1
    best_difference = None
    for index in range(start, len(records)):
        difference = abs(records[index].header_stamp_us - target_us)
        if records[index].header_stamp_us > target_us + half_gap_us:
            break
        if difference <= half_gap_us and (best_difference is None or difference < best_difference):
            best_index = index
            best_difference = difference
    return best_index


def group_synced_frames(streams, half_gap_us):
    """Greedily make one-to-one complete groups following the official analyzer."""
    names = sorted(streams)
    ordered = {name: sorted(streams[name], key=lambda item: item.header_stamp_us) for name in names}
    positions = {name: 0 for name in names}
    groups = []
    unmatched = 0

    while all(positions[name] < len(ordered[name]) for name in names):
        anchor_name = min(names, key=lambda name: ordered[name][positions[name]].header_stamp_us)
        anchor = ordered[anchor_name][positions[anchor_name]]
        selected = {anchor_name: positions[anchor_name]}
        for name in names:
            if name == anchor_name:
                continue
            index = nearest_index(ordered[name], positions[name], anchor.header_stamp_us, half_gap_us)
            if index < 0:
                unmatched += 1
                positions[anchor_name] += 1
                break
            selected[name] = index
        else:
            groups.append({name: ordered[name][index] for name, index in selected.items()})
            for name, index in selected.items():
                unmatched += index - positions[name]
                positions[name] = index + 1

    unmatched += sum(len(ordered[name]) - positions[name] for name in names)
    return groups, unmatched


def percentage(numerator, denominator):
    return 100.0 * numerator / denominator if denominator else 0.0


def print_sensor_report(sensor, records_by_key, half_gap_us, threshold_us):
    print(f"\n========== Sensor: {sensor.upper()} ==========")
    streams = {
        camera: records
        for (camera, stream), records in records_by_key.items()
        if stream == sensor
    }
    for camera, records in sorted(streams.items()):
        print(f"  {camera}: {len(records)} frames")
    if len(streams) < 2:
        print("  Fewer than 2 cameras, cannot evaluate sync, skip")
        return

    groups, unmatched = group_synced_frames(streams, half_gap_us)
    ranges = [
        max(record.header_stamp_us for record in group.values())
        - min(record.header_stamp_us for record in group.values())
        for group in groups
    ]
    total = len(groups) + unmatched
    print(f"  Complete groups: {len(groups)}/{total} ({percentage(len(groups), total):.1f}%), unmatched: {unmatched}")
    if not ranges:
        print("  No complete groups within the matching interval")
        return
    abnormal = sum(value >= threshold_us for value in ranges)
    print(
        "  Precision range: "
        f"{min(ranges)} ~ {max(ranges)} us, mean {statistics.mean(ranges):.1f} us, "
        f"median {statistics.median(ranges):.1f} us"
    )
    print(f"  Abnormal: {abnormal}/{len(ranges)} ({percentage(abnormal, len(ranges)):.1f}%)")


def print_intra_device_report(records_by_key, half_gap_us, threshold_us):
    print("\n========== Intra-device checks (depth-color) ==========")
    cameras = sorted({camera for camera, _ in records_by_key})
    for camera in cameras:
        color = records_by_key.get((camera, "color"), [])
        depth = records_by_key.get((camera, "depth"), [])
        print(f"  {camera}: color {len(color)} frames, depth {len(depth)} frames")
        if not color or not depth:
            print("    Missing color or depth stream, skip")
            continue
        groups, unmatched = group_synced_frames({"color": color, "depth": depth}, half_gap_us)
        differences = [
            group["depth"].header_stamp_us - group["color"].header_stamp_us
            for group in groups
        ]
        total = len(groups) + unmatched
        print(f"    pairs: {len(groups)}/{total} ({percentage(len(groups), total):.1f}%), unmatched: {unmatched}")
        if not differences:
            print("    No color-depth pairs within the matching interval")
            continue
        absolute = [abs(value) for value in differences]
        abnormal = sum(value >= threshold_us for value in absolute)
        print(f"    header diff: {min(differences)} ~ {max(differences)} us (depth - color)")
        print(
            f"    absolute diff: mean {statistics.mean(absolute):.1f} us, "
            f"median {statistics.median(absolute):.1f} us, max {max(absolute)} us"
        )
        print(f"    Abnormal: {abnormal}/{len(differences)} ({percentage(abnormal, len(differences)):.1f}%)")


def parse_arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "capture_path",
        nargs="?",
        default=".",
        help="capture directory or timestamps.csv (default: current directory)",
    )
    parser.add_argument("--fps", type=float, help="nominal frame rate for matching")
    parser.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD_US, help="abnormal threshold in us")
    return parser.parse_args(argv)


def main(argv=None):
    try:
        arguments = parse_arguments(argv)
        if arguments.fps is not None and not 0 < arguments.fps < 1000:
            raise ValueError("--fps must be greater than 0 and less than 1000")
        if arguments.threshold <= 0:
            raise ValueError("--threshold must be greater than 0")
        csv_path = resolve_input_csv(Path(arguments.capture_path))
        records_by_key = load_capture(csv_path)
        detected_fps = infer_fps(records_by_key)
        fps = arguments.fps or detected_fps or DEFAULT_FPS
    except (OSError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    half_gap_us = 1_000_000.0 / fps / 2.0
    source = "--fps" if arguments.fps else ("auto-detected from receive_stamp_us" if detected_fps else "fallback")
    print("=" * 60)
    print("Capture timestamp precision analysis")
    print(f"  Input CSV  : {csv_path}")
    print("  Time base  : ROS header_stamp_us")
    print(f"  Frame rate : {fps:.2f} fps ({source})")
    print(f"    - same-stream frames are matched within half a frame interval ({half_gap_us:.0f} us)")
    print(f"  Threshold  : {arguments.threshold:g} us")
    print("    - a complete group with max-min header timestamp range >= threshold is Abnormal")
    print("=" * 60)

    stream_rates = {
        f"{camera}/{stream}": stream_fps(records)
        for (camera, stream), records in records_by_key.items()
    }
    valid_rates = {name: value for name, value in stream_rates.items() if value is not None}
    if arguments.fps is None and len(valid_rates) > 1 and max(valid_rates.values()) / min(valid_rates.values()) > 1.2:
        rate_text = ", ".join(f"{name}={value:.2f}" for name, value in sorted(valid_rates.items()))
        print(f"Warning: materially different stream rates detected ({rate_text}); use --fps for a focused comparison.")

    print_sensor_report("color", records_by_key, half_gap_us, arguments.threshold)
    print_sensor_report("depth", records_by_key, half_gap_us, arguments.threshold)
    print_intra_device_report(records_by_key, half_gap_us, arguments.threshold)
    return 0


if __name__ == "__main__":
    sys.exit(main())
