# OB ROS2 GMSL Project Handoff

## Purpose

This document summarizes the completed work, the current state, known hardware facts, and the remaining implementation steps for the next AI or engineer.

The end goal is a ROS 2 Jazzy workflow that receives three externally hardware-triggered GMSL cameras, publishes their depth/color image streams through Orbbec ROS 2 driver nodes, and records per-image timestamps and optional raw image bytes using a separate collector package.

---

## 1. Completed work

### 1.1 Local Orbbec ROS 2 documentation archive

The Orbbec ROS 2 documentation was archived locally:

```text
D:\Data\robotPackage\ob_tools\资料\OrbbecSDK_ROS2_docs
```

It has been used to inspect Orbbec ROS 2 installation, multi-camera, GMSL, synchronization, and parameter guidance.

### 1.2 Existing C++ SDK code was compared with the ROS 2 design

Existing direct Orbbec C++ SDK code:

```text
D:\Data\robotPackage\ob_tools\src
```

The direct SDK uses richer per-frame time information, including concepts represented by:

```cpp
hwTimestampUs
globalTimestampUs
sysTimestampUs
frameNumber
```

In contrast, the ROS collector consumes:

```cpp
sensor_msgs::msg::Image
```

The key fields used from each ROS Image message are:

```cpp
message->header.stamp
message->header.frame_id
message->width
message->height
message->encoding
message->step
message->data
```

Important conclusion:

- `message->header.stamp` and `message->data` belong to the same ROS `Image` message, therefore the collector associates the timestamp with the same image frame it receives.
- `header.stamp` is not automatically guaranteed to be a cross-camera global hardware timestamp. Its meaning depends on the installed Orbbec driver timestamp/time-domain configuration.
- The collector also records the callback receive time. This is useful to observe ROS/DDS/CPU/storage delay, but it is not an exposure or trigger time.

---

## 2. Created ROS 2 collector package

Package source directory:

```text
D:\Data\robotPackage\ob_tools\OB ROS2
```

Package name:

```text
ob_ros2_timestamp_collector
```

Current structure:

```text
OB ROS2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/ob_ros2_timestamp_collector/
│   ├── capture_record.hpp
│   └── collector_core.hpp
├── src/
│   ├── collector_core.cpp
│   └── timestamp_collector_node.cpp
├── config/
│   ├── collector.yaml
│   └── gmsl_scheme_a_collector.yaml
├── launch/
│   ├── timestamp_collector.launch.py
│   └── gmsl_scheme_a_capture.launch.py
└── test/
    ├── test_capture_record.cpp
    └── test_collector_core.cpp
```

This package is intentionally **collector-only**:

- subscribes to already-published ROS 2 `sensor_msgs/msg/Image` topics;
- writes timestamps and image metadata to CSV;
- optionally writes raw `Image.data` bytes to `.bin` files;
- does **not** launch or configure cameras;
- does **not** generate hardware triggers;
- does **not** implement multi-camera frame matching;
- does **not** calculate synchronization accuracy;
- does **not** modify the original direct-SDK C++ implementation under `src`.

---

## 3. Collector implementation details

### 3.1 ROS subscriber node

File:

```text
OB ROS2/src/timestamp_collector_node.cpp
```

Responsibilities:

- reads subscription definitions from YAML;
- creates one `sensor_msgs/msg/Image` subscription per configured topic;
- extracts per-message timestamp, metadata, and optional raw image bytes;
- sends records to `CollectorCore`;
- supports an automatic stop timer through `duration_sec`;
- supports a per-stream maximum frame count;
- defaults to sensor-data-oriented QoS.

Each subscription entry follows this exact format:

```text
camera_name|stream_type|topic
```

`stream_type` must be either:

```text
color
depth
```

The core timestamp logic is equivalent to:

```cpp
record.header_stamp_us =
  stamp_to_microseconds(message->header.stamp.sec, message->header.stamp.nanosec);

record.receive_stamp_us = get_clock()->now().nanoseconds() / 1000LL;
record.data_size = message->data.size();
```

Definitions:

| Field | Meaning |
| --- | --- |
| `header_stamp_us` | Microsecond form of `Image.header.stamp`, supplied upstream by the ROS driver. |
| `receive_stamp_us` | The collector node's callback execution time; reflects transport/scheduling/load and is not a hardware trigger timestamp. |
| `data_size` | Number of raw image bytes in `Image.data`. |

When `save_images: true`, the node copies `Image.data` and forwards it to the background persistence path.

### 3.2 Persistence core

File:

```text
OB ROS2/src/collector_core.cpp
```

Responsibilities:

- creates a timestamped capture directory;
- creates `timestamps.csv`;
- records one CSV row per received image;
- optionally saves original image bytes as `.bin`;
- uses a bounded background writer queue for raw image writes;
- records how many image-save jobs are dropped when the queue is full.

CSV columns:

```text
camera_name,stream_type,frame_index,header_stamp_us,receive_stamp_us,frame_id,width,height,encoding,step,data_size,image_path
```

Expected output layout:

```text
<output_dir>/
└── capture_YYYYmmdd_HHMMSS/
    ├── timestamps.csv
    └── images/                         # only when save_images=true
        ├── camera_305_01/
        │   ├── depth/
        │   │   └── 00000000.bin
        │   └── color/
        │       └── 00000000.bin
        ├── camera_305_02/
        └── camera_335lg_01/
```

Raw `.bin` files are not PNG/JPEG files. They contain unchanged `Image.data` bytes. Interpret each file using its CSV record:

```text
encoding
width
height
step
data_size
```

If the background queue is full:

- timestamp and metadata CSV rows are still written;
- the CSV row's `image_path` is empty;
- `dropped_image_jobs` increments;
- this is collector persistence loss, not proof of a ROS camera-topic frame loss.

### 3.3 Tests

Current tests:

```text
OB ROS2/test/test_capture_record.cpp
OB ROS2/test/test_collector_core.cpp
```

They cover:

- timestamp conversion;
- CSV escaping;
- CSV file creation;
- `.bin` creation;
- raw persisted bytes matching input bytes.

---

## 4. Jetson ROS 2 environment and device discovery

Target environment:

```text
Ubuntu 24.04
ROS 2 Jazzy
Jetson / arm64
```

Installed ROS packages:

```text
orbbec_camera
orbbec_camera_msgs
orbbec_description
```

The Orbbec device discovery command was run successfully:

```bash
ros2 run orbbec_camera list_devices_node
```

Three GMSL2 devices were found:

| Planned ROS namespace | Model | Serial number | GMSL port |
| --- | --- | --- | --- |
| `camera_305_01` | Gemini 305g | `CV3T561000B5` | `gmsl2-5` |
| `camera_305_02` | Gemini 305g | `CV3T561000H0` | `gmsl2-4` |
| `camera_335lg_01` | Gemini 335Lg | `CPBG1630011D` | `gmsl2-6` |

All were enumerated as:

```text
Connection: GMSL2
usb connect type: GMSL2
```

This confirms:

- Orbbec ROS 2 driver installation works;
- the Orbbec lower-level device stack detects all three cameras;
- the known GMSL port identifiers can be used in later driver binding if supported by the installed Jazzy driver.

---

## 5. External trigger topology

The shared hardware trigger is already generated outside ROS by the Jetson GPIO trigger driver:

```bash
echo 30 | sudo tee /sys/kernel/debug/gpio_trigger/framerate
```

Conceptual data flow:

```text
Jetson GPIO trigger driver
    ↓
Shared external trigger at approximately 30 Hz
    ↓
Gemini 305g / gmsl2-5
Gemini 305g / gmsl2-4
Gemini 335Lg / gmsl2-6
    ↓
Three namespaced Orbbec ROS 2 driver nodes
    ↓
Six color/depth Image topics
    ↓
ob_ros2_timestamp_collector
    ↓
timestamps.csv + optional raw .bin image files
```

Rules for future work:

- the new ROS launch must **not** write to GPIO sysfs;
- the new ROS launch must **not** generate another GMSL/SoC trigger;
- the Orbbec nodes need only be configured for the compatible passive external-trigger behavior supported by the actual installed driver;
- do not enable a second trigger mechanism unless the physical topology explicitly requires it.

---

## 6. Current launch/configuration state

### Existing collector launch

```text
OB ROS2/launch/timestamp_collector.launch.py
```

This starts only:

```text
timestamp_collector_node
```

### Existing combined wrapper

```text
OB ROS2/launch/gmsl_scheme_a_capture.launch.py
```

Its current behavior is:

```text
include user-supplied driver launch
+
start timestamp_collector_node
```

It does **not** know or configure:

```text
gmsl2-4
gmsl2-5
gmsl2-6
CV3T561000B5
CV3T561000H0
CPBG1630011D
camera stream profiles
external trigger synchronization mode
time-domain parameters
```

Therefore it does not itself start the three cameras.

### Existing GMSL collector configuration

```text
OB ROS2/config/gmsl_scheme_a_collector.yaml
```

It currently contains placeholder example topics and only two cameras. It must be replaced with six actual topic paths after confirming the custom driver launch output.

The desired eventual form is:

```yaml
subscriptions:
  - "camera_305_01|depth|<actual depth topic>"
  - "camera_305_01|color|<actual color topic>"
  - "camera_305_02|depth|<actual depth topic>"
  - "camera_305_02|color|<actual color topic>"
  - "camera_335lg_01|depth|<actual depth topic>"
  - "camera_335lg_01|color|<actual color topic>"
```

---

## 7. Important mismatch discovered

Archived/older Orbbec documentation mentions GMSL-specific launch files such as:

```text
multi_gmsl_camera_synced.launch.py
multi_gmsl_camera.launch.py
```

However, the installed Jazzy package launch directory did not contain those files.

Relevant installed launch files include:

```text
gemini305.launch.py
gemini345_lg.launch.py
multi_camera.launch.py
multi_camera_synced.launch.py
orbbec_camera.launch.py
orbbec_multicamera.launch.py
```

Therefore:

- do not copy or assume old GMSL launch filenames;
- do not write parameters based only on archived documentation;
- inspect the installed Jazzy driver files first;
- create a project-owned launch/configuration that targets the actual installed driver contract.

---

## 8. Required remaining implementation

### 8.1 First: inspect the installed Jazzy driver contract

Before creating runnable camera YAML files, execute on the Jetson and save the complete output:

```bash
source /opt/ros/jazzy/setup.bash

PKG="$(ros2 pkg prefix orbbec_camera)/share/orbbec_camera"

echo "===== Gemini 305 launch ====="
sed -n '1,260p' "$PKG/launch/gemini305.launch.py"

echo "===== Gemini 345 LG launch ====="
sed -n '1,260p' "$PKG/launch/gemini345_lg.launch.py"

echo "===== Generic multi-camera launch ====="
sed -n '1,320p' "$PKG/launch/multi_camera.launch.py"

echo "===== Installed YAML files ====="
find "$PKG" -type f -name '*.yaml' -print
```

Then inspect any referenced YAML files. Confirm these exact facts from the installed Jazzy code/configuration:

1. **Device binding parameter**
   - Is it `usb_port`, serial number, `device_num`, or another setting?
   - Prefer serial-number binding if the installed driver supports it; otherwise use the enumerated GMSL port.

2. **External-trigger mode**
   - Exact parameter name and accepted value for passive external hardware triggering.
   - Whether `frames_per_trigger` is supported and required.
   - Any mandatory GMSL sync or stream-switching setting.

3. **Timestamp configuration**
   - Whether `time_domain` exists.
   - Its valid global-time value.
   - Whether `enable_sync_host_time` exists and must be disabled for the selected domain.

4. **Image stream configuration**
   - Exact color/depth enable flags.
   - Valid resolution, format, and frame-rate parameters for Gemini 305g.
   - Valid resolution, format, and frame-rate parameters for Gemini 335Lg.
   - Whether the two models require different profile definitions.

5. **Actual topic paths**
   - Start one namespaced driver node and use `ros2 topic list`, `ros2 topic info`, and `ros2 topic echo --once`.
   - Use the actual published depth/color topic names in the collector YAML.

### 8.2 Then: add a custom three-camera driver launch

Create:

```text
OB ROS2/launch/three_gmsl_external_trigger.launch.py
```

Expected behavior:

- launch exactly three `orbbec_camera/orbbec_camera_node` processes;
- assign independent namespaces and node names:

```text
/camera_305_01
/camera_305_02
/camera_335lg_01
```

- load one parameter YAML per camera;
- never generate trigger signals;
- never write GPIO sysfs;
- keep all camera selection, streams, sync mode, and timestamp-domain settings in parameter YAML files.

### 8.3 Then: create per-camera driver YAML files

Create:

```text
OB ROS2/config/gmsl/camera_305_01.yaml
OB ROS2/config/gmsl/camera_305_02.yaml
OB ROS2/config/gmsl/camera_335lg_01.yaml
```

Required binding:

```text
camera_305_01 → CV3T561000B5 / gmsl2-5
camera_305_02 → CV3T561000H0 / gmsl2-4
camera_335lg_01 → CPBG1630011D / gmsl2-6
```

Requirements:

- use only parameter names and values verified from the installed Jazzy package;
- enable only required streams, initially color + depth;
- use compatible passive external-trigger mode;
- set one frame per trigger only if supported/required by the confirmed contract;
- configure global/shared timestamp domain only if verified;
- keep Gemini 335Lg configuration independent from Gemini 305g where profiles or settings differ.

### 8.4 Then: update collector config, combined launch, and README

Update:

```text
OB ROS2/config/gmsl_scheme_a_collector.yaml
OB ROS2/launch/gmsl_scheme_a_capture.launch.py
OB ROS2/README.md
```

Required changes:

- six verified collector subscriptions;
- default combined launch should start the project-owned three-camera driver launch and the collector;
- remove references to the missing old `multi_gmsl_camera_synced.launch.py`;
- document ROS 2 Jazzy build steps;
- document external-trigger responsibilities;
- document single-camera → two-camera → three-camera → collector validation order.

The collector C++ sources should remain unchanged:

```text
OB ROS2/src/timestamp_collector_node.cpp
OB ROS2/src/collector_core.cpp
```

---

## 9. Jetson build and validation workflow

### Build package

Place package source in a ROS workspace, preferably without the display-name directory containing spaces:

```bash
mkdir -p ~/ros2_ws/src
cp -a "OB ROS2" ~/ros2_ws/src/ob_ros2_timestamp_collector

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ob_ros2_timestamp_collector
source install/setup.bash
```

### Validate in stages

1. Enable existing external trigger:

   ```bash
   echo 30 | sudo tee /sys/kernel/debug/gpio_trigger/framerate
   ```

2. Launch only `camera_305_01`.
   - Confirm correct device binding.
   - Confirm color/depth topics.
   - Confirm image headers and metadata.
   - Check approximately 30 Hz output.

3. Add `camera_305_02` and repeat checks.

4. Add `camera_335lg_01` and confirm all six streams have unique namespaces and stable output.

5. Start collector with conservative settings:

   ```yaml
   duration_sec: 10
   save_images: false
   ```

   Verify `timestamps.csv` has rows for all expected streams.

6. Enable raw output:

   ```yaml
   save_images: true
   ```

   Verify:

   - every non-empty CSV `image_path` points to an existing `.bin`;
   - each `.bin` size matches the corresponding `data_size` column;
   - shutdown reports zero dropped image-save jobs at the selected profiles/frequency.

If image-save jobs are dropped, adjust storage speed or `writer_queue_capacity`. Do not use `receive_stamp_us` alone as a hardware synchronization accuracy metric.

---

## 10. Constraints and safety notes

- Do not modify:

  ```text
  D:\Data\robotPackage\ob_tools\src
  ```

  unless the user explicitly requests changes to the original C++ direct-SDK program.

- Do not commit Git changes unless explicitly requested.

- The local Windows machine does not have ROS 2/colcon setup for realistic package builds. Build and hardware validation must happen on the Jetson.

- The Git working tree already contained many unrelated deleted files. Do not restore, remove, or commit unrelated changes.

- `OB ROS2/` is currently untracked by Git.

- Current collector scope deliberately excludes:

  ```text
  multi-camera timestamp matching
  synchronization quality computation
  hardware trigger control
  direct Orbbec SDK camera control
  ```
