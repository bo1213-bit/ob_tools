#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 timestamp synchronization analyzer for Orbbec multi-camera topics.

The logic mirrors the C++ SyncAnalyzer:
  - use global timestamps for matching (ROS Image.header.stamp when time_domain=global)
  - ignore warmup frames
  - compare same-camera depth/color
  - compare cross-camera depth/depth and color/color
  - compute multi-camera max(timestamp)-min(timestamp)
"""

from __future__ import print_function

import csv
import math
import os
import threading

import rospy
from sensor_msgs.msg import Image


STREAM_TOPIC = {
    "color": "color/image_raw",
    "depth": "depth/image_raw",
}


class FrameStamp(object):
    __slots__ = ["camera", "stream", "stamp_us", "arrival_us", "seq", "frame_id"]

    def __init__(self, camera, stream, stamp_us, arrival_us, seq, frame_id):
        self.camera = camera
        self.stream = stream
        self.stamp_us = int(stamp_us)
        self.arrival_us = int(arrival_us)
        self.seq = int(seq)
        self.frame_id = frame_id


def parse_csv_param(value):
    if isinstance(value, list):
        return [str(x).strip() for x in value if str(x).strip()]
    return [x.strip() for x in str(value).split(",") if x.strip()]


def time_to_us(stamp):
    return int(stamp.secs) * 1000000 + int(stamp.nsecs) // 1000


def mean_stddev(values):
    if not values:
        return 0.0, 0.0
    mean = sum(float(v) for v in values) / float(len(values))
    sq_sum = sum((float(v) - mean) * (float(v) - mean) for v in values)
    return mean, math.sqrt(sq_sum / float(len(values)))


def stats(values):
    if not values:
        return {
            "count": 0,
            "min": 0,
            "max": 0,
            "mean": 0.0,
            "stddev": 0.0,
        }
    mean, stddev = mean_stddev(values)
    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": mean,
        "stddev": stddev,
    }


class TimestampSyncAnalyzer(object):
    def __init__(self):
        self.camera_names = parse_csv_param(
            rospy.get_param("~camera_names", "camera_01,camera_02")
        )
        self.streams = parse_csv_param(rospy.get_param("~streams", "color,depth"))
        self.match_threshold_us = int(rospy.get_param("~match_threshold_us", 5000))
        self.warmup_frames = int(rospy.get_param("~warmup_frames", 3))
        self.output_dir = rospy.get_param("~output_dir", "/tmp/ob_ros1_sync")
        self.duration_sec = float(rospy.get_param("~duration_sec", 0.0))
        self.analyze_on_shutdown = bool(rospy.get_param("~analyze_on_shutdown", True))

        self.frames = {}
        self.seen_counts = {}
        self.lock = threading.Lock()
        self.subscribers = []
        self.raw_file = None
        self.raw_writer = None
        self.summary_written = False

        for camera in self.camera_names:
            self.frames[camera] = {}
            self.seen_counts[camera] = {}
            for stream in self.streams:
                self.frames[camera][stream] = []
                self.seen_counts[camera][stream] = 0

        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        self.raw_csv_path = os.path.join(self.output_dir, "raw_timestamps.csv")
        self.summary_csv_path = os.path.join(self.output_dir, "sync_diffs.csv")
        self.raw_file = open(self.raw_csv_path, "w")
        self.raw_writer = csv.writer(self.raw_file)
        self.raw_writer.writerow(["camera", "stream", "stamp_us", "arrival_us", "seq", "frame_id"])
        self.raw_file.flush()

    def start(self):
        rospy.loginfo("timestamp sync analyzer cameras=%s streams=%s threshold=%dus warmup=%d",
                      self.camera_names, self.streams, self.match_threshold_us, self.warmup_frames)
        for camera in self.camera_names:
            for stream in self.streams:
                topic = "/{}/{}".format(camera.strip("/"), STREAM_TOPIC.get(stream, stream))
                sub = rospy.Subscriber(
                    topic,
                    Image,
                    self._callback,
                    callback_args=(camera, stream),
                    queue_size=200,
                    buff_size=2 ** 24,
                )
                self.subscribers.append(sub)
                rospy.loginfo("subscribe %s", topic)

        if self.duration_sec > 0.0:
            rospy.Timer(rospy.Duration(self.duration_sec), self._duration_done, oneshot=True)

    def _duration_done(self, _event):
        rospy.loginfo("duration_sec reached, writing analysis and shutting down")
        self.write_analysis()
        rospy.signal_shutdown("duration_sec reached")

    def _callback(self, msg, args):
        camera, stream = args
        stamp_us = time_to_us(msg.header.stamp)
        arrival_us = time_to_us(rospy.Time.now())
        seq = getattr(msg.header, "seq", 0)
        frame_id = getattr(msg.header, "frame_id", "")

        with self.lock:
            self.seen_counts[camera][stream] += 1
            seen = self.seen_counts[camera][stream]
            if seen <= self.warmup_frames:
                return

            fs = FrameStamp(camera, stream, stamp_us, arrival_us, seq, frame_id)
            self.frames[camera][stream].append(fs)
            self.raw_writer.writerow([camera, stream, stamp_us, arrival_us, seq, frame_id])
            if len(self.frames[camera][stream]) % 30 == 0:
                self.raw_file.flush()

    def match_and_diff(self, a_frames, b_frames):
        diffs = []
        ref_stamps = []
        if not a_frames or not b_frames:
            return diffs, ref_stamps

        b_stamps = [f.stamp_us for f in b_frames]
        for af in a_frames:
            best_dist = None
            best_stamp = None
            for b_stamp in b_stamps:
                dist = abs(af.stamp_us - b_stamp)
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_stamp = b_stamp
            if best_dist is not None and best_dist < self.match_threshold_us:
                diffs.append(af.stamp_us - best_stamp)
                ref_stamps.append(af.stamp_us)
        return diffs, ref_stamps

    def multi_device_match(self, per_camera_frames):
        valid = [(camera, frames) for camera, frames in per_camera_frames if frames]
        if len(valid) < 2:
            return [], []

        ref_camera, ref_frames = min(valid, key=lambda item: len(item[1]))
        others = [(camera, frames) for camera, frames in valid if camera != ref_camera]
        group_diffs = []
        ref_stamps = []

        for rf in ref_frames:
            group = [rf.stamp_us]
            matched = True
            for _camera, frames in others:
                best_dist = None
                best_stamp = None
                for frame in frames:
                    dist = abs(rf.stamp_us - frame.stamp_us)
                    if best_dist is None or dist < best_dist:
                        best_dist = dist
                        best_stamp = frame.stamp_us
                if best_dist is None or best_dist >= self.match_threshold_us:
                    matched = False
                    break
                group.append(best_stamp)
            if matched:
                group_diffs.append(max(group) - min(group))
                ref_stamps.append(rf.stamp_us)
        return group_diffs, ref_stamps

    def snapshot_frames(self):
        with self.lock:
            copied = {}
            for camera in self.camera_names:
                copied[camera] = {}
                for stream in self.streams:
                    copied[camera][stream] = list(self.frames[camera][stream])
            if self.raw_file:
                self.raw_file.flush()
            return copied

    def write_analysis(self):
        if self.summary_written:
            return
        self.summary_written = True

        frames = self.snapshot_frames()
        rows = []
        report = []

        def add_result(label, device_i, device_j, stream_label, diffs, refs):
            for diff, ref in zip(diffs, refs):
                rows.append([label, device_i, device_j, stream_label, diff, ref])
            s = stats([abs(d) for d in diffs] if label != "multi_device" else diffs)
            report.append((label, device_i, device_j, stream_label, s))

        # Same-camera depth vs color.
        if "depth" in self.streams and "color" in self.streams:
            for camera in self.camera_names:
                diffs, refs = self.match_and_diff(frames[camera]["depth"], frames[camera]["color"])
                add_result("cross_stream", camera, camera, "depth+color", diffs, refs)

        # Cross-camera same stream.
        for stream in ["depth", "color"]:
            if stream not in self.streams:
                continue
            for i in range(len(self.camera_names)):
                for j in range(i + 1, len(self.camera_names)):
                    cam_i = self.camera_names[i]
                    cam_j = self.camera_names[j]
                    diffs, refs = self.match_and_diff(frames[cam_i][stream], frames[cam_j][stream])
                    add_result("cross_device", cam_i, cam_j, stream, diffs, refs)

        # Multi-camera max-min per stream.
        for stream in ["depth", "color"]:
            if stream not in self.streams:
                continue
            per_camera = [(camera, frames[camera][stream]) for camera in self.camera_names]
            diffs, refs = self.multi_device_match(per_camera)
            add_result("multi_device", "all", "all", stream + "_all", diffs, refs)

        with open(self.summary_csv_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow([
                "comparison_type",
                "device_i",
                "device_j",
                "stream_label",
                "diff_us",
                "reference_stamp_us",
            ])
            writer.writerows(rows)

        rospy.loginfo("raw timestamps CSV: %s", self.raw_csv_path)
        rospy.loginfo("sync diff CSV: %s", self.summary_csv_path)
        rospy.loginfo("========== Orbbec ROS1 Timestamp Sync Report ==========")
        for label, dev_i, dev_j, stream_label, s in report:
            rospy.loginfo(
                "%s %s %s->%s count=%d min=%dus max=%dus mean=%.1fus stddev=%.1fus",
                label,
                stream_label,
                dev_i,
                dev_j,
                s["count"],
                s["min"],
                s["max"],
                s["mean"],
                s["stddev"],
            )

    def shutdown(self):
        if self.analyze_on_shutdown:
            try:
                self.write_analysis()
            except Exception as exc:  # pylint: disable=broad-except
                rospy.logwarn("failed to write timestamp analysis: %s", exc)
        if self.raw_file:
            try:
                self.raw_file.flush()
                self.raw_file.close()
            except Exception:
                pass


def main():
    rospy.init_node("ros_timestamp_sync_analyzer")
    analyzer = TimestampSyncAnalyzer()
    rospy.on_shutdown(analyzer.shutdown)
    analyzer.start()
    rospy.spin()


if __name__ == "__main__":
    main()
