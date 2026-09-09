import argparse
import csv
import os
import re
import statistics
import sys

DEFAULT_FRAME_RATE = None              # None = auto-detect; grouping tolerance = 1e6 / fps / 2 us
DEFAULT_TSP_RANGE_THRESHOLD = 5000.0  # us; in-group range >= this -> abnormal
DEFAULT_TIMESTAMP_SOURCE = "auto"     # global | device | auto
DEFAULT_FRAME_NUM_SOURCE = "auto"     # hw | sw | auto (drop detection)

CSV_NAME_RE = re.compile(r"^sync_(depth|color)_dev(\d+)_(.+)\.csv$", re.IGNORECASE)


# --------------------------------------------------------------------------- #
# Data structures
# --------------------------------------------------------------------------- #
class FrameRecord:
    __slots__ = ("row_id", "sw_frame_num", "hw_frame_num",
                 "system_ts", "device_ts", "global_ts")

    def __init__(self, row_id, sw_frame_num, hw_frame_num,
                 system_ts, device_ts, global_ts):
        self.row_id = row_id
        self.sw_frame_num = sw_frame_num
        self.hw_frame_num = hw_frame_num
        self.system_ts = system_ts
        self.device_ts = device_ts
        self.global_ts = global_ts

    def ts(self, source):
        return self.global_ts if source == "global" else self.device_ts


class DeviceFrames:
    def __init__(self, index, sn, sensor):
        self.index = index
        self.sn = sn
        self.sensor = sensor
        self.frames = []
        self._sorted = []
        self._ts = []
        self.hw_valid = True
        self.sw_valid = True

    def sort(self, ts_source):
        self.frames.sort(key=lambda r: r.ts(ts_source))
        self._sorted = self.frames
        self._ts = [r.ts(ts_source) for r in self._sorted]

    def is_ts_valid(self, ts_source):
        if not self.frames:
            return False
        return any(r.ts(ts_source) != 0 for r in self.frames)

    def check_frame_num_validity(self):
        if self.frames:
            self.hw_valid = any(r.hw_frame_num != -1 for r in self.frames)
            self.sw_valid = any(r.sw_frame_num != -1 for r in self.frames)

    def nearest_from(self, target_ts, start_idx, max_diff):
        """Find the frame nearest to target_ts at index >= start_idx. Returns (idx, diff) or (-1, None)."""
        arr = self._ts
        n = len(arr)
        last_below = -1
        j = start_idx
        while j < n and arr[j] < target_ts:
            if arr[j] >= target_ts - max_diff:
                last_below = j
            j += 1
        below_diff = (target_ts - arr[last_below]) if last_below != -1 else None
        above_diff = (arr[j] - target_ts) if j < n and arr[j] <= target_ts + max_diff else None
        if below_diff is None and above_diff is None:
            return -1, None
        if below_diff is None:
            return j, above_diff
        if above_diff is None:
            return last_below, below_diff
        return (last_below, below_diff) if below_diff <= above_diff else (j, above_diff)


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #
def discover_csv(data_dir):
    result = {"depth": {}, "color": {}}
    found = 0
    for name in os.listdir(data_dir):
        m = CSV_NAME_RE.match(name)
        if not m:
            continue
        sensor = m.group(1).lower()
        index = int(m.group(2))
        sn = m.group(3)
        df = DeviceFrames(index, sn, sensor)
        path = os.path.join(data_dir, name)
        try:
            with open(path, "r", encoding="utf-8-sig", newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row.get("global_ts_us") in (None, "") and row.get("device_ts_us") in (None, ""):
                        continue
                    df.frames.append(FrameRecord(
                        int(row["row_id"]) if row.get("row_id") not in (None, "") else 0,
                        int(row["sw_frame_num"]) if row.get("sw_frame_num") not in (None, "") else -1,
                        int(row["hw_frame_num"]) if row.get("hw_frame_num") not in (None, "") else -1,
                        int(row["system_ts_us"]) if row.get("system_ts_us") not in (None, "") else 0,
                        int(row["device_ts_us"]) if row.get("device_ts_us") not in (None, "") else 0,
                        int(row["global_ts_us"]) if row.get("global_ts_us") not in (None, "") else 0,
                    ))
        except (OSError, ValueError, KeyError) as e:
            print("WARN: failed to read %s: %s" % (name, e))
            continue
        df.check_frame_num_validity()
        result[sensor][index] = df
        found += 1
    return result, found


def resolve_ts_source(requested, devices, sensor, lines):
    """Resolve the time base from the request and data validity across all devices."""
    global_valid = all(d.is_ts_valid("global") for d in devices)
    device_valid = all(d.is_ts_valid("device") for d in devices)
    chosen = requested
    notes = []
    if requested == "auto":
        if global_valid:
            chosen = "global"
        elif device_valid:
            chosen = "device"
            notes.append("Auto-degraded to device timestamps (global_ts_us all 0, device may not support global timestamp)")
        else:
            chosen = "device"
            notes.append("WARN: both global and device timestamps are abnormal; still matching on device")
    elif requested == "global" and not global_valid:
        invalid = [d.index for d in devices if not d.is_ts_valid("global")]
        notes.append("WARN: global selected but dev%s has all-zero global_ts_us" % invalid)
    elif requested == "device" and not device_valid:
        notes.append("WARN: device timestamps are abnormal")
    for n in notes:
        lines.append("  " + n)
    if chosen == "device":
        lines.append("  [%s] RISK: matching on DEVICE timestamps - device clocks are per-device;"
                     " without clock sync during capture, cross-device diffs include clock"
                     " offset/drift, indicative only." % sensor.upper())
    return chosen


def group_synced_frames(devices, ts_source, half_gap_us):
    """Greedy grouping: each round picks the earliest remaining frame as the anchor
    and matches all devices within the half-frame interval.
    Returns (matched_groups, failed); each group is {dev_index: FrameRecord}.
    failed is a list of unmatched anchor (dev, FrameRecord) pairs."""
    devices = sorted(devices, key=lambda d: d.index)
    n = len(devices)
    ptr = [0] * n
    groups = []
    failed = []

    while True:
        anchor = -1
        anchor_ts = None
        for i in range(n):
            if ptr[i] < len(devices[i]._ts):
                t = devices[i]._ts[ptr[i]]
                if anchor_ts is None or t < anchor_ts:
                    anchor_ts = t
                    anchor = i
        if anchor == -1:
            break

        match = [None] * n
        match[anchor] = ptr[anchor]
        ok = True
        for i in range(n):
            if i == anchor:
                continue
            idx, diff = devices[i].nearest_from(anchor_ts, ptr[i], half_gap_us)
            if idx < 0:
                ok = False
                break
            match[i] = idx
        if not ok:
            failed.append((devices[anchor], devices[anchor]._sorted[ptr[anchor]]))
            ptr[anchor] += 1
            continue

        group = {}
        for i in range(n):
            # frames skipped between ptr[i] and match[i] were unmatched for this device
            for k in range(ptr[i], match[i]):
                failed.append((devices[i], devices[i]._sorted[k]))
            group[devices[i].index] = devices[i]._sorted[match[i]]
            ptr[i] = match[i] + 1
        groups.append(group)

    return groups, failed


def write_failed_csv(path, failed):
    """One row per unmatched frame (anchor that found no partner within tolerance,
    or a frame skipped during a match); the frame itself was captured normally."""
    with open(path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sn", "rowId", "swNum", "hwNum", "systemUs", "deviceUs", "globalUs"])
        for dev, frame in failed:
            w.writerow([dev.sn, frame.row_id, frame.sw_frame_num, frame.hw_frame_num,
                        frame.system_ts, frame.device_ts, frame.global_ts])


def write_matched_csv(path, devices, groups, ts_source):
    devices = sorted(devices, key=lambda d: d.index)
    global_ok = all(d.is_ts_valid("global") for d in devices)
    device_ok = all(d.is_ts_valid("device") for d in devices)
    header = ["groupId", "diffGlobalUs", "diffDeviceUs"]
    for d in devices:
        prefix = "dev%d" % d.index
        header += [prefix + "SwNum",
                   prefix + "HwNum",
                   prefix + "SystemUs",
                   prefix + "GlobalUs",
                   prefix + "DeviceUs"]
    with open(path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for gid, group in enumerate(groups):
            global_list = [group[d.index].global_ts for d in devices]
            device_list = [group[d.index].device_ts for d in devices]
            g_range = (max(global_list) - min(global_list)) if global_ok else ""
            d_range = (max(device_list) - min(device_list)) if device_ok else ""
            row = [gid, g_range, d_range]
            for d in devices:
                r = group[d.index]
                row += [r.sw_frame_num, r.hw_frame_num, r.system_ts, r.global_ts, r.device_ts]
            w.writerow(row)


def detect_fps(sensors_data):
    """Detect frame rate from the median of per-stream median frame intervals.
    Returns fps or None if there is not enough data."""
    medians = []
    for sensor in sensors_data:
        for df in sensors_data[sensor].values():
            ts = [r.system_ts for r in df.frames]
            gaps = [b - a for a, b in zip(ts, ts[1:]) if 0 < b - a < 1000000]
            if len(gaps) >= 5:
                medians.append(statistics.median(gaps))
    if not medians:
        return None
    return round(1e6 / statistics.median(medians), 2)


def make_view(df, index, ts_source):
    v = DeviceFrames(index, df.sn, df.sensor)
    v.frames = list(df.frames)
    v.hw_valid = df.hw_valid
    v.sw_valid = df.sw_valid
    v.sort(ts_source)
    return v


def resolve_per_device_ts(requested, streams):
    """Resolve the pairing time base for the depth/color streams of ONE device."""
    global_valid = all(s.is_ts_valid("global") for s in streams)
    if requested == "auto":
        return ("global", "") if global_valid else ("device", " - global_ts_us all 0")
    if requested == "global" and not global_valid:
        return "global", " - WARN global_ts_us all 0"
    return "device", ""


def detect_drops(frames, source):
    """Detect dropped frames via frame-number gaps. Returns (missing_nums, valid_count, first_num).
    Note: drops before the first recorded frame cannot be detected (no gap to observe)."""
    attr = "hw_frame_num" if source == "hw" else "sw_frame_num"
    nums = sorted(getattr(r, attr) for r in frames if getattr(r, attr) != -1)
    missing = []
    for a, b in zip(nums, nums[1:]):
        if b > a + 1:
            missing.extend(range(a + 1, b))
    return missing, len(nums), (nums[0] if nums else None)


def analyze_per_device(sensors_data, ts_source, frame_num_source, half_gap_us, per_dir, lines):
    """Single-device checks per device: depth-color pairing diff + per-stream drop stats."""
    lines.append("\n========== Intra-device checks (depth-color, frame drops) ==========")
    indices = sorted(set(sensors_data["depth"]) | set(sensors_data["color"]))
    drop_rows = []
    files_written = 0

    for idx in indices:
        depth = sensors_data["depth"].get(idx)
        color = sensors_data["color"].get(idx)
        sn = (depth or color).sn
        streams = [s for s in (depth, color) if s and s.frames]

        fn_src = frame_num_source
        fn_note = ""
        if frame_num_source == "auto":
            fn_src = "hw" if all(s.hw_valid for s in streams) else "sw"
            if fn_src == "sw":
                fn_note = " (hw_frame_num all -1)"
        elif frame_num_source == "hw" and not all(s.hw_valid for s in streams):
            lines.append("    dev%d(%s) WARN: --frame-num-source hw but hw_frame_num all -1, drop detection unreliable" % (idx, sn))

        lines.append("")
        if depth and color and depth.frames and color.frames:
            src, ts_note = resolve_per_device_ts(ts_source, [depth, color])
            lines.append("    dev%d(%s)" % (idx, sn))
            lines.append("        ts source       : %s%s" % (src, ts_note.replace(" - ", " (") + (")" if ts_note else "")))
            lines.append("        frame num       : %s%s" % (fn_src.upper(), fn_note))
            lines.append("        frames          : depth %d, color %d" % (len(depth.frames), len(color.frames)))

            groups, failed = group_synced_frames([make_view(depth, 0, src), make_view(color, 1, src)], src, half_gap_us)
            lines.append("        pairs           : %d (unmatched %d)" % (len(groups), len(failed)))
            if groups:
                if depth.is_ts_valid("global") and color.is_ts_valid("global"):
                    g_diffs = [g[0].global_ts - g[1].global_ts for g in groups]
                    lines.append("        global diff     : %d ~ %d us (depth - color)" % (min(g_diffs), max(g_diffs)))
                else:
                    lines.append("        global diff     : n/a (global_ts_us invalid)")
                device_valid = depth.is_ts_valid("device") and color.is_ts_valid("device")
                if src == "device" and device_valid:
                    d_diffs = [g[0].device_ts - g[1].device_ts for g in groups]
                    lines.append("        device diff     : %d ~ %d us (depth - color)" % (min(d_diffs), max(d_diffs)))
                global_ok = depth.is_ts_valid("global") and color.is_ts_valid("global")
                device_ok = depth.is_ts_valid("device") and color.is_ts_valid("device")
                os.makedirs(per_dir, exist_ok=True)
                path = os.path.join(per_dir, "per_device_dev%d_%s.csv" % (idx, sn))
                with open(path, "w", encoding="utf-8-sig", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["groupId", "diffGlobalUs", "diffDeviceUs",
                                "depthSwNum", "depthHwNum", "depthSystemUs", "depthGlobalUs", "depthDeviceUs",
                                "colorSwNum", "colorHwNum", "colorSystemUs", "colorGlobalUs", "colorDeviceUs"])
                    for gid, g in enumerate(groups):
                        d, c = g[0], g[1]
                        w.writerow([gid,
                                    (d.global_ts - c.global_ts) if global_ok else "",
                                    (d.device_ts - c.device_ts) if device_ok else "",
                                    d.sw_frame_num, d.hw_frame_num, d.system_ts, d.global_ts, d.device_ts,
                                    c.sw_frame_num, c.hw_frame_num, c.system_ts, c.global_ts, c.device_ts])
                lines.append("        CSV             : %s" % path)
                files_written += 1
        else:
            lines.append("    dev%d(%s)" % (idx, sn))
            lines.append("        frame num       : %s%s" % (fn_src.upper(), fn_note))
            lines.append("        pairing         : skip (only one stream has data)")

        starts = []
        parts = []
        for label, s in (("depth", depth), ("color", color)):
            if not s or not s.frames:
                continue
            missing, valid, first_num = detect_drops(s.frames, fn_src)
            if missing:
                parts.append("%s %d (%.2f%%)" % (label, len(missing), 100.0 * len(missing) / (valid + len(missing))))
            else:
                parts.append("%s 0" % label)
            if first_num is not None and first_num > 1:
                starts.append("%s at %sNum %d" % (label, "hw" if fn_src == "hw" else "sw", first_num))
            for m in missing:
                drop_rows.append((sn, label, fn_src, m))
        if parts:
            lines.append("        dropped frames  : " + ", ".join(parts) + "  (positions in Drops CSV)")
        if starts:
            lines.append("        first csv row    : " + ", ".join(starts) + "  (earlier frames not captured, not counted as drops)")

    if drop_rows:
        os.makedirs(per_dir, exist_ok=True)
        path = os.path.join(per_dir, "per_device_drops.csv")
        with open(path, "w", encoding="utf-8-sig", newline="") as f:
            w = csv.writer(f)
            w.writerow(["sn", "sensor", "frameNumSource", "missingFrameNum"])
            for r in drop_rows:
                w.writerow(r)
        lines.append("    Drops CSV: %s (%d frames)" % (path, len(drop_rows)))
        files_written += 1

    return files_written


def analyze_sensor(sensor, devices_map, ts_source, half_gap_us, threshold, lines):
    lines.append("\n========== Sensor: %s ==========" % sensor.upper())
    devices = sorted(devices_map.values(), key=lambda x: x.index)
    if len(devices) < 2:
        lines.append("    Fewer than 2 devices, cannot evaluate sync, skip")
        return None, []

    for d in devices:
        lines.append("    dev%d(%s): %d frames" % (d.index, d.sn, len(d.frames)))

    duration_s = 0.0
    all_ts = [r.ts(ts_source) for d in devices for r in d.frames]
    if all_ts:
        duration_s = (max(all_ts) - min(all_ts)) / 1e6
        if duration_s > 0:
            min_frames = min(len(d.frames) for d in devices)
            lines.append("    Duration: %.3f s  (~%.2f fps)" % (duration_s, min_frames / duration_s))

    groups, failed = group_synced_frames(devices, ts_source, half_gap_us)
    total_anchors = len(groups) + len(failed)
    completeness = 100.0 * len(groups) / total_anchors if total_anchors else 0.0
    lines.append("    Completeness      : %.1f%% (%d groups, %d unmatched)" % (completeness, len(groups), len(failed)))

    if not groups:
        lines.append("    (no complete groups, cannot evaluate)")
        return None, failed

    ranges = []
    for g in groups:
        ts_list = [g[d.index].ts(ts_source) for d in devices]
        ranges.append(max(ts_list) - min(ts_list))
    abnormal = sum(1 for r in ranges if r >= threshold)
    lines.append("    Abnormal (>= %.0fus): %.1f%% (%d / %d)" %
                 (threshold, 100.0 * abnormal / len(groups), abnormal, len(groups)))

    return groups, failed


# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(
        description="Multi-device sync timestamp CSV analyzer",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("data_dir", nargs="?", default=None,
                        help="Directory with sync_*_dev*_<SN>.csv (default: current directory)")
    parser.add_argument("--fps", type=float, default=DEFAULT_FRAME_RATE,
                        help="Frame rate; grouping tolerance = half frame (default: auto-detect from data)")
    parser.add_argument("--threshold", type=float, default=DEFAULT_TSP_RANGE_THRESHOLD,
                        help="Abnormal in-group range threshold in us (default %.0f)" % DEFAULT_TSP_RANGE_THRESHOLD)
    parser.add_argument("--ts-source", choices=["global", "device", "auto"],
                        default=DEFAULT_TIMESTAMP_SOURCE,
                        help="Time base: global/device/auto (default %s)" % DEFAULT_TIMESTAMP_SOURCE)
    parser.add_argument("--frame-num-source", choices=["hw", "sw", "auto"],
                        default=DEFAULT_FRAME_NUM_SOURCE,
                        help="Frame number source for drop detection: hw/sw/auto (default %s)" % DEFAULT_FRAME_NUM_SOURCE)
    parser.add_argument("--output", default=None,
                        help="Output directory base for CSVs (default: <data_dir>, writes analysis_multi_device/ and analysis_per_device/)")
    args = parser.parse_args()

    data_dir = os.path.abspath(args.data_dir) if args.data_dir else os.getcwd()
    if not os.path.isdir(data_dir):
        print("ERROR: directory does not exist: %s" % data_dir)
        return 1

    sensors_data, found = discover_csv(data_dir)
    if found == 0:
        print("ERROR: no sync_*_dev*_<SN>.csv found under %s" % data_dir)
        return 1

    threshold = args.threshold
    ts_source = args.ts_source

    fps = args.fps
    fps_note = ""
    if fps is None:
        fps = detect_fps(sensors_data)
        if fps is None:
            fps = 30.0
            fps_note = "  (auto-detect failed, too few frames, fallback to 30)"
        else:
            fps_note = "  (auto-detected from median frame interval)"
    if fps <= 0 or fps >= 1000:
        print("ERROR: invalid frameRate=%.1f" % fps)
        return 1

    half_gap_us = 1000000.0 / fps / 2.0
    base_out = args.output or data_dir
    out_dir = os.path.join(base_out, "analysis_multi_device")
    os.makedirs(out_dir, exist_ok=True)

    print("=" * 60)
    print("Multi-device sync CSV analysis")
    print("  Data dir   : %s" % data_dir)
    print("  Frame rate : %.2f fps%s" % (fps, fps_note))
    print("    - frames from all devices are grouped when their timestamps fall within")
    print("      half a frame interval (%.0f us); a wrong fps gives a wrong tolerance" % half_gap_us)
    print("  Threshold  : %.0f us" % threshold)
    print("    - a matched group whose max-min timestamp range reaches this value")
    print("      is counted as Abnormal")
    print("  Time base  : %s" % ts_source)
    print("    - timestamps used for matching; global = global_ts_us, device = device_ts_us,")
    print("      auto = global when available, degrades to device otherwise")
    print("  Frame num  : %s" % args.frame_num_source)
    print("    - frame number used for drop detection; hw = hw_frame_num, sw = sw_frame_num,")
    print("      auto = hw when available, degrades to sw otherwise (see 'frame num' per device)")
    print("=" * 60)

    lines = []
    for sensor in ("depth", "color"):
        devices = sensors_data[sensor]
        if not devices:
            lines.append("\n========== Sensor: %s ==========" % sensor.upper())
            lines.append("    No data, skip")
            continue
        chosen_ts = resolve_ts_source(ts_source, list(devices.values()), sensor, lines)
        for d in devices.values():
            d.sort(chosen_ts)
        groups, failed = analyze_sensor(sensor, devices, chosen_ts, half_gap_us, threshold, lines)
        if failed:
            fail_csv = os.path.join(out_dir, "sync_failed_%s.csv" % sensor)
            write_failed_csv(fail_csv, failed)
            lines.append("    Failed CSV: %s (%d unmatched)" % (fail_csv, len(failed)))
        if groups:
            out_csv = os.path.join(out_dir, "sync_matched_%s.csv" % sensor)
            write_matched_csv(out_csv, list(devices.values()), groups, chosen_ts)
            lines.append("    Matched CSV: %s (%d groups)" % (out_csv, len(groups)))

    per_dir = os.path.join(base_out, "analysis_per_device")
    analyze_per_device(sensors_data, ts_source, args.frame_num_source, half_gap_us, per_dir, lines)

    print("\n" + "\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())