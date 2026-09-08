# OB ROS2 Timestamp Collector

## Purpose

This package provides a project-owned ROS 2 launch for three externally triggered Orbbec GMSL cameras and a downstream timestamp collector. The collector subscribes to `sensor_msgs/msg/Image` topics, writes timestamp/image metadata to CSV, and can optionally save raw image byte buffers.

It does **not** match frames, assess synchronization quality, write GPIO sysfs, or generate hardware trigger signals.

## Hardware-trigger topology

The Jetson GPIO wiring provides the shared external trigger. All cameras must receive that same trigger, and the ROS launch configures each camera passively:

- `sync_mode: hardware_triggering`
- `frames_per_trigger: 1`
- `trigger_out_enabled: false`
- `time_domain: global`
- `enable_sync_host_time: false`
- `enable_point_cloud: false`

The two Gemini 305g cameras additionally set `enable_gmsl_trigger: false` so the driver does not create a second GMSL/SoC trigger. The Gemini 335Lg configuration intentionally does **not** set that parameter because its installed Jazzy launch does not expose it.

Do not enable another trigger source while the external GPIO trigger is connected.

## Camera mapping

| ROS namespace | Model | Serial number | GMSL port |
|---|---|---|---|
| `camera_305_01` | Gemini 305g | `CV3T561000B5` | `gmsl2-5` |
| `camera_305_02` | Gemini 305g | `CV3T561000H0` | `gmsl2-4` |
| `camera_335lg_01` | Gemini 335Lg | `CPBG1630011D` | `gmsl2-6` |

The camera launch binds devices by serial number, never by discovery order or GMSL port.

## Project launch files

- `launch/three_gmsl_external_trigger.launch.py` starts one independent component container per camera using `orbbec_camera::OBCameraNodeDriver`.
- `config/gmsl/camera_305_01.yaml`, `camera_305_02.yaml`, and `camera_335lg_01.yaml` hold device-specific parameters.
- `launch/gmsl_scheme_a_capture.launch.py` starts all three cameras plus `timestamp_collector_node`.
- `config/gmsl_scheme_a_collector.yaml` configures six expected color/depth subscriptions.

The camera YAML files deliberately leave stream profiles at driver/device defaults. No resolution, pixel format, or frame-rate profile is forced by this package.

## Build (ROS 2 Jazzy)

Create or use a ROS 2 workspace. The source directory can contain spaces, but package discovery is clearer when it is copied under the package name:

```bash
mkdir -p ~/ros2_ws/src
cp -a "OB ROS2" ~/ros2_ws/src/ob_ros2_timestamp_collector
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ob_ros2_timestamp_collector
source install/setup.bash
```

The installed `orbbec_camera` package and `rclcpp_components` must be available in the same Jazzy environment.

## Run cameras only

Start the GPIO trigger generator before the cameras, then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ob_ros2_timestamp_collector three_gmsl_external_trigger.launch.py
```

Expected camera image topics are:

```text
/camera_305_01/depth/image_raw
/camera_305_01/color/image_raw
/camera_305_02/depth/image_raw
/camera_305_02/color/image_raw
/camera_335lg_01/depth/image_raw
/camera_335lg_01/color/image_raw
```

The `camera_name` driver parameter is explicitly set for every component. This is required because the Orbbec driver uses it to construct its image topic prefix; assigning only a ROS node namespace leaves all three cameras publishing to conflicting root topics such as `/color/image_raw` and `/depth/image_raw`.

Confirm the actual topics after launching:

```bash
ros2 node list
ros2 topic list | grep -E '/camera_(305_01|305_02|335lg_01)/(color|depth)/image_raw$'
ros2 topic hz /camera_305_01/color/image_raw
ros2 topic hz /camera_305_02/color/image_raw
ros2 topic hz /camera_335lg_01/color/image_raw
```

The Gemini 335Lg currently produces color data at approximately 15 Hz under the existing hardware-trigger setup; this is expected hardware behavior. Verify its depth topic independently as well.

## Run cameras and collector together

Start the external GPIO trigger generator before starting this launch. The combined launch starts all three cameras and the timestamp collector, using `config/gmsl_scheme_a_collector.yaml` by default:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ob_ros2_timestamp_collector gmsl_scheme_a_capture.launch.py
```

The default collector configuration records for `duration_sec: 30`, then the collector exits automatically. The camera containers can continue printing logs; wait specifically for the collector's completion line:

```text
[timestamp_collector_node]: CSV: /home/mscape/ob_ros2_gmsl_capture/capture_YYYYmmdd_HHMMSS/timestamps.csv
[timestamp_collector_node]: process has finished cleanly
```

The `CSV:` line is the authoritative location for that capture. Do not interrupt the collector before it prints that line.

Override the collector configuration without changing camera settings:

```bash
ros2 launch ob_ros2_timestamp_collector gmsl_scheme_a_capture.launch.py \
  collector_config_file:=/path/to/gmsl_scheme_a_collector.yaml
```

Set `duration_sec: 0` to run until ROS is stopped with `Ctrl-C`.

### Use a fixed capture parent directory

The GMSL collector configuration is intended to use this persistent parent directory:

```text
/home/mscape/ob_ros2_gmsl_capture
```

Create it once:

```bash
mkdir -p ~/ob_ros2_gmsl_capture
```

In `~/ros2_ws/src/ob_ros2_timestamp_collector/config/gmsl_scheme_a_collector.yaml`, set:

```yaml
output_dir: "/home/mscape/ob_ros2_gmsl_capture"
```

After changing a YAML file under `src/`, rebuild this package before launching. The launch uses the installed package share directory, so an already-built workspace can otherwise keep using the previous installed configuration:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ob_ros2_timestamp_collector
source install/setup.bash
```

Verify the configuration actually used by `ros2 launch` before collecting:

```bash
grep -n 'output_dir' \
  "$(ros2 pkg prefix ob_ros2_timestamp_collector)/share/ob_ros2_timestamp_collector/config/gmsl_scheme_a_collector.yaml"
```

It must print:

```text
output_dir: "/home/mscape/ob_ros2_gmsl_capture"
```

Every completed collection then has its own timestamped directory, so previous data is never overwritten:

```text
~/ob_ros2_gmsl_capture/
├── capture_YYYYmmdd_HHMMSS/
│   └── timestamps.csv
└── capture_YYYYmmdd_HHMMSS/
    └── timestamps.csv
```

## Configure collection

For a generic deployment, edit `config/collector.yaml`. For the three-camera GMSL deployment, use `config/gmsl_scheme_a_collector.yaml`.

Each subscription uses this format:

```text
camera_name|stream_type|topic
```

`stream_type` must be `depth` or `color`; each `camera_name|stream_type` pair must be unique. If `ros2 topic list` shows different paths, update the collector YAML before starting a capture.

## Output

Each run creates a directory such as `capture_YYYYmmdd_HHMMSS` below `output_dir`:

```text
capture_YYYYmmdd_HHMMSS/
├── timestamps.csv
└── images/                         # only when save_images=true
    └── <camera>/<stream>/<index>.bin
```

The CSV schema is:

```text
camera_name,stream_type,frame_index,header_stamp_us,receive_stamp_us,frame_id,width,height,encoding,step,data_size,image_path
```

- `header_stamp_us` comes from `Image.header.stamp`. It is a cross-camera comparison candidate only when the driver publishes the configured global time domain.
- `receive_stamp_us` records ROS receipt time, not camera exposure synchronization.
- `frame_index` is generated independently for each configured stream.
- `frame_id` is the ROS coordinate-frame identifier, not a sequence number.

With `save_images: true`, the original `Image.data` buffer is saved unchanged as a `.bin` file. Use the CSV `encoding`, `width`, `height`, `step`, and `data_size` fields to interpret it. No PNG conversion is performed.

Raw-image writes use a bounded background queue. When it fills, the collector still writes timestamp/metadata rows, leaves `image_path` empty, and reports dropped image-save jobs on shutdown.

## Validation checklist

Run the package tests after building:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ob_ros2_timestamp_collector
colcon test --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
colcon test-result --verbose
```

Then validate on the Jetson with the external trigger active:

1. Launch `three_gmsl_external_trigger.launch.py` and verify all three camera nodes/containers start.
2. Verify all six color/depth image topics and their output rates; check the 335Lg depth stream in addition to its known ~15 Hz color stream.
3. Launch `gmsl_scheme_a_capture.launch.py`, inspect the resulting `timestamps.csv`, and only then enable `save_images: true` for raw-image persistence.
4. Observe whether the Gemini 335Lg reports `gmsl_trigger_fps_ illegal: 3000`; do not add unverified 335Lg trigger parameters solely to suppress that warning.

## Analyze offline timestamp precision

Each completed capture directory already contains `timestamps.csv`. To analyze the newest completed collection, open a second terminal and run:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

cd "$(ls -td ~/ob_ros2_gmsl_capture/capture_*/ | head -n 1)"
python3 "$(ros2 pkg prefix ob_ros2_timestamp_collector)/share/ob_ros2_timestamp_collector/scripts/analyze_capture_sync.py"
```

The analyzer automatically reads `./timestamps.csv` and prints its report in the terminal. It is read-only, so it is safe to run after the collector has printed its final `CSV:` line.

To make a capture folder self-contained, copy the analyzer into that folder once and then run it with no arguments:

```bash
cd ~/ob_ros2_gmsl_capture/capture_YYYYmmdd_HHMMSS
cp "$(ros2 pkg prefix ob_ros2_timestamp_collector)/share/ob_ros2_timestamp_collector/scripts/analyze_capture_sync.py" .
python3 analyze_capture_sync.py
```

List all completed captures with:

```bash
ls -ltd ~/ob_ros2_gmsl_capture/capture_*/
```

You can also run the installed analyzer without copying it, either against a specific capture directory or a specific CSV file:

```bash
python3 "$(ros2 pkg prefix ob_ros2_timestamp_collector)/share/ob_ros2_timestamp_collector/scripts/analyze_capture_sync.py" \
  ~/ob_ros2_gmsl_capture/capture_YYYYmmdd_HHMMSS

python3 "$(ros2 pkg prefix ob_ros2_timestamp_collector)/share/ob_ros2_timestamp_collector/scripts/analyze_capture_sync.py" \
  ~/ob_ros2_gmsl_capture/capture_YYYYmmdd_HHMMSS/timestamps.csv \
  --fps 30 --threshold 2000
```

The terminal report evaluates cross-camera `COLOR` and `DEPTH` groups, then each camera's depth/color pairs. It follows the official half-frame matching rule: frames must be within half a nominal frame interval, and each complete cross-camera group's precision is the maximum minus minimum timestamp in microseconds. A range at or above `2000 us` is reported as abnormal by default.

The synchronization comparison source is **ROS `header_stamp_us`**, not an Orbbec SDK global/device timestamp. `receive_stamp_us` is used only to estimate a nominal rate when `--fps` is omitted. The collector CSV has no hardware or software camera frame counters, device timestamp, or SDK global timestamp, so this tool cannot diagnose camera-side frame-counter drops or evaluate those unavailable time bases. It is read-only: it does not adjust external triggering and does not address the separate 335Lg rate issue.
