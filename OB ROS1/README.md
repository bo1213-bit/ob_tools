# OB ROS1：奥比中光多相机时间戳同步示例

这个文件夹里的代码只关注 **ROS1 版本的多相机同步过程**，同步方式按照当前 C++ 程序的逻辑来写：

- 不使用 `primary + secondary_synced` 作为默认方案；
- 所有相机都使用 `hardware_triggering`；
- 外部硬触发频率通过 `/sys/kernel/debug/gpio_trigger/framerate` 控制；
- ROS 图像时间戳使用 Orbbec 的 `global` 时间域；
- 用 ROS 订阅图像 topic，统计相机之间的时间戳差。

对应 C++ 逻辑：

- `DataCollector::configureSyncMode()`：所有设备设成硬触发；
- `writeTriggerFramerate()`：写 `/sys/kernel/debug/gpio_trigger/framerate`；
- `resetTimestampAndSyncClock()`：启用 global timestamp；
- `collectFrames()`：先关触发、启动相机、预热、采集；
- `SyncAnalyzer`：用 global timestamp 做 depth/color、跨相机、多相机同步分析。

---

## 目录结构

```text
OB ROS1/
  launch/
    multi_camera_hw_trigger.launch
  scripts/
    ob_trigger_controller.py
    ros_timestamp_sync_analyzer.py
  README.md
```

---

## 1. 文件说明

### `launch/multi_camera_hw_trigger.launch`

ROS1 多相机硬触发 launch 示例。

默认配置两个相机：

```text
camera_01 -> usb_port 2-1
camera_02 -> usb_port 2-2
```

需要根据你的机器修改：

```xml
<arg name="camera1_usb_port" default="2-1"/>
<arg name="camera2_usb_port" default="2-2"/>
```

核心同步参数：

```xml
<arg name="sync_mode" value="hardware_triggering"/>
<arg name="trigger_out_enabled" value="false"/>
<arg name="frames_per_trigger" value="1"/>
<arg name="depth_delay_us" value="0"/>
<arg name="color_delay_us" value="0"/>
<arg name="trigger2image_delay_us" value="0"/>
<arg name="trigger_out_delay_us" value="0"/>
<arg name="time_domain" value="global"/>
<arg name="enable_sync_host_time" value="false"/>
<arg name="enable_frame_sync" value="true"/>
```

这些参数对应 C++ 里的：

```cpp
cfg.syncMode = OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING;
cfg.triggerOutEnable = true/false;
cfg.framesPerTrigger = 1;
cfg.depthDelayUs = 0;
cfg.colorDelayUs = 0;
cfg.trigger2ImageDelayUs = 0;
cfg.triggerOutDelayUs = 0;
device->enableGlobalTimestamp(true);
```

ROS 里这里把 `trigger_out_enabled` 默认设为 `false`，因为触发信号来自外部 GPIO/PWM，不由相机输出。

---

### `scripts/ob_trigger_controller.py`

外部触发控制节点，等价于 C++ 里的 `writeTriggerFramerate()`。

启动流程：

1. 写 `0` 到 `/sys/kernel/debug/gpio_trigger/framerate`，先关触发；
2. 等待 `start_delay_sec`，让 ROS 相机节点启动稳定；
3. 写 `trigger_hz`，开始外部触发。

参数：

```text
~trigger_node: /sys/kernel/debug/gpio_trigger/framerate
~trigger_hz: 30
~start_delay_sec: 2.0
~stop_on_shutdown: false
```

注意：`stop_on_shutdown` 默认是 `false`。因为你的 C++ 里有一个很重要的经验：**停止时先停相机 pipeline，再关 trigger**，否则某些平台上可能导致 tegra camera 驱动异常或重启。

如果你确认当前平台不会有这个问题，可以设置：

```bash
stop_trigger_on_shutdown:=true
```

---

### `scripts/ros_timestamp_sync_analyzer.py`

ROS 图像时间戳同步分析节点。

订阅 topic：

```text
/camera_01/color/image_raw
/camera_01/depth/image_raw
/camera_02/color/image_raw
/camera_02/depth/image_raw
```

默认使用 `sensor_msgs/Image.header.stamp` 作为同步时间戳。因此 Orbbec ROS1 必须配置：

```text
time_domain=global
enable_sync_host_time=false
```

它会输出两个 CSV：

```text
/tmp/ob_ros1_sync/raw_timestamps.csv
/tmp/ob_ros1_sync/sync_diffs.csv
```

`raw_timestamps.csv` 格式：

```text
camera,stream,stamp_us,arrival_us,seq,frame_id
```

`sync_diffs.csv` 格式：

```text
comparison_type,device_i,device_j,stream_label,diff_us,reference_stamp_us
```

统计类型和 C++ `SyncAnalyzer` 对齐：

```text
cross_stream: 同一相机 depth vs color
cross_device: 跨相机 depth vs depth / color vs color
multi_device: 所有相机同一流 max(timestamp)-min(timestamp)
```

---

## 2. 使用前准备

### 2.1 确认 Orbbec ROS1 wrapper 可用

需要能正常运行类似：

```bash
roslaunch orbbec_camera gemini_330_series.launch
```

如果你的相机不是 Gemini 330 系列，修改 launch 里的：

```xml
<arg name="orbbec_launch_file" default="$(find orbbec_camera)/launch/gemini_330_series.launch"/>
```

换成你的相机型号对应 launch 文件。

---

### 2.2 查询 USB port

用 Orbbec 官方工具或 ROS wrapper 工具查询相机 USB port，例如：

```bash
rosrun orbbec_camera list_ob_devices.sh
```

或者：

```bash
rosrun orbbec_camera list_devices_node
```

然后修改：

```xml
<arg name="camera1_usb_port" default="2-1"/>
<arg name="camera2_usb_port" default="2-2"/>
```

---

### 2.3 提高 USB buffer

多相机建议提高 USB buffer：

```bash
echo 512 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

---

### 2.4 确认 trigger 节点权限

触发控制节点需要写：

```bash
/sys/kernel/debug/gpio_trigger/framerate
```

如果普通用户无权限，可以先手动测试：

```bash
echo 0 | sudo tee /sys/kernel/debug/gpio_trigger/framerate
echo 30 | sudo tee /sys/kernel/debug/gpio_trigger/framerate
```

如果必须 sudo，建议用 root 启动 ROS，或给该 debugfs 节点配置合适权限。

---

## 3. 推荐启动方式

### 方式 A：全自动启动 trigger controller

把 `scripts/` 里的两个脚本放进某个 ROS package 的 `scripts/` 目录，并赋予执行权限：

```bash
chmod +x ob_trigger_controller.py ros_timestamp_sync_analyzer.py
```

假设 package 名为 `ob_ros1_sync`，运行：

```bash
roslaunch "OB ROS1/launch/multi_camera_hw_trigger.launch" \
  helper_pkg:=ob_ros1_sync \
  camera1_usb_port:=2-1 \
  camera2_usb_port:=2-2 \
  fps:=30 \
  trigger_hz:=30
```

流程是：

1. 相机节点启动；
2. `ob_trigger_controller.py` 先写 0；
3. 等 `trigger_start_delay_sec`；
4. 写 `trigger_hz`；
5. analyzer 订阅图像并记录时间戳。

---

### 方式 B：更接近 C++ 的手动安全流程

如果你担心停止顺序导致驱动问题，推荐这个方式：

#### 1. 先关外部触发

```bash
echo 0 | sudo tee /sys/kernel/debug/gpio_trigger/framerate
```

#### 2. 启动相机和分析节点，但不启动 helper

```bash
roslaunch "OB ROS1/launch/multi_camera_hw_trigger.launch" \
  start_helper_nodes:=false \
  camera1_usb_port:=2-1 \
  camera2_usb_port:=2-2 \
  fps:=30
```

#### 3. 相机节点稳定后，打开外部触发

```bash
echo 30 | sudo tee /sys/kernel/debug/gpio_trigger/framerate
```

#### 4. 采集完成后，先 Ctrl+C 停 ROS 相机

等相机节点退出。

#### 5. 最后再关外部触发

```bash
echo 0 | sudo tee /sys/kernel/debug/gpio_trigger/framerate
```

这和 C++ 的“先停 pipeline，再关 trigger”更接近。

---

## 4. 单独运行分析节点

如果不通过 launch 启动 analyzer，可以手动运行：

```bash
rosrun ob_ros1_sync ros_timestamp_sync_analyzer.py \
  _camera_names:=camera_01,camera_02 \
  _streams:=color,depth \
  _match_threshold_us:=5000 \
  _warmup_frames:=3 \
  _output_dir:=/tmp/ob_ros1_sync
```

如果只采 color：

```bash
rosrun ob_ros1_sync ros_timestamp_sync_analyzer.py \
  _camera_names:=camera_01,camera_02 \
  _streams:=color
```

---

## 5. 对比 C++ 结果

C++ 原始 CSV：

```text
deviceIndex,streamType,hwTimestampUs,globalTimestampUs,sysTimestampUs,frameNumber
```

ROS 版原始 CSV：

```text
camera,stream,stamp_us,arrival_us,seq,frame_id
```

ROS 版的 `stamp_us` 应该对应 C++ 里的 `globalTimestampUs`，前提是 Orbbec ROS1 的：

```text
time_domain=global
```

生效。

重点看这些指标：

```text
cross_device color camera_01 -> camera_02
cross_device depth camera_01 -> camera_02
multi_device color_all
multi_device depth_all
```

如果同步正常，`diff_us` 应该稳定在一个比较小的范围内。

---

## 6. 增加到 3/4 个相机

在 `multi_camera_hw_trigger.launch` 里复制一个 camera include block，修改：

```xml
<arg name="camera3_name" default="camera_03"/>
<arg name="camera3_usb_port" default="2-3"/>
```

对应 include 里改：

```xml
<arg name="camera_name" value="$(arg camera3_name)"/>
<arg name="usb_port" value="$(arg camera3_usb_port)"/>
<arg name="frame_timestamp_csv_file" value="$(arg output_dir)/orbbec_camera_03_frame_timestamps.csv"/>
```

同时 analyzer 的 `camera_names` 要包含新相机：

```xml
<param name="camera_names" value="camera_01,camera_02,camera_03"/>
```

---

## 7. 常见问题

### 7.1 没有图像

检查：

```bash
rostopic list | grep image_raw
```

确认 topic 是否是：

```text
/camera_01/color/image_raw
/camera_01/depth/image_raw
```

如果 topic 名不一样，需要改 `ros_timestamp_sync_analyzer.py` 里的 topic 规则，或者后续给脚本加显式 topic 参数。

---

### 7.2 时间戳差很大

优先检查：

```text
time_domain 是否为 global
enable_sync_host_time 是否为 false
所有相机 fps 是否完全一致
所有相机是否真的进入 hardware_triggering
外部 trigger 频率是否稳定
USB bandwidth 是否足够
```

---

### 7.3 trigger 节点无法写入

报错类似：

```text
cannot write trigger node /sys/kernel/debug/gpio_trigger/framerate
```

说明权限不足或节点不存在。检查：

```bash
ls -l /sys/kernel/debug/gpio_trigger/framerate
```

必要时用 sudo 或 root 运行。

---

### 7.4 ROS launch 找不到 helper package

这个文件夹是独立资料/代码文件夹，不是完整 catkin package。

如果要让 launch 自动启动脚本，需要把 scripts 放进某个 ROS package，例如：

```text
catkin_ws/src/ob_ros1_sync/scripts/
```

然后：

```bash
chmod +x catkin_ws/src/ob_ros1_sync/scripts/*.py
catkin_make
source devel/setup.bash
```

再运行 launch，并设置：

```bash
helper_pkg:=ob_ros1_sync
```

如果不想建 package，就用：

```bash
start_helper_nodes:=false
```

然后手动运行脚本。
