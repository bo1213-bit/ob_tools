# ros2_sync_tool 修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标:** 修复 ros2_sync_tool 的时间戳来源和架构，将相机启动与分析解耦。

**架构:** 删除相机启动逻辑，只保留分析节点。硬件时间戳从 `header.stamp` 读取，系统到达时间从 `this->now()` 读取。用户手动运行官方 launch 启动相机，再运行分析节点。

**技术栈:** ROS2 (rclcpp), sensor_msgs, C++17, Python (launch files), YAML

## 全局约束

- 不修改 `sync_analyzer_core.cpp/h`、`frame_stamp.h`、`CMakeLists.txt`
- 所有 camera_name 必须与官方 launch 中的保持一致
- 分析节点运行 `duration_sec` 秒后自动退出
- 不执行 git 操作，用户自己管理版本

---

### Task 1: 修复时间戳来源

**修改文件:**
- `src/ros2_sync_analyzer_node.cpp:89-91`

**接口:**
- 读取: `sensor_msgs::msg::Image::ConstSharedPtr` → `.header.stamp`
- 产出: `FrameStamp.sysTimestampUs` 从 `this->now()` 获取微秒值，`FrameStamp.hwTimestampUs` 从 `header.stamp` 获取微秒值

- [ ] **将 `imageCallback` 中的时间戳赋值修改为两个不同来源**

找到 `src/ros2_sync_analyzer_node.cpp` 第 89-91 行：

```cpp
        // Extract timestamp from header.stamp
        fs.hwTimestampUs  = static_cast<int64_t>(msg->header.stamp.sec) * 1000000LL
                          + static_cast<int64_t>(msg->header.stamp.nanosec) / 1000LL;
        fs.sysTimestampUs = fs.hwTimestampUs;  // Same source for now
```

改为：

```cpp
        // Hardware/global timestamp from header.stamp
        fs.hwTimestampUs  = static_cast<int64_t>(msg->header.stamp.sec) * 1000000LL
                          + static_cast<int64_t>(msg->header.stamp.nanosec) / 1000LL;
        // System arrival time from ROS callback
        fs.sysTimestampUs = static_cast<int64_t>(this->now().nanoseconds()) / 1000LL;
```

---

### Task 2: 删除旧的相机 launch 文件

**删除文件:**
- `launch/multi_camera_sync.launch.py`

- [ ] **直接删除该文件**（相机启动由官方 launch 负责）

---

### Task 3: 创建纯分析 launch 文件

**新建文件:**
- `launch/sync_analyzer.launch.py`

**接口:**
- 读取: `config/cameras.yaml` → analyzer 配置参数
- 产出: 启动 `sync_analyzer_node` 并传入参数

- [ ] **创建 `launch/sync_analyzer.launch.py`**

```python
#!/usr/bin/env python3
"""
Launch file for sync_analyzer_node (analysis only, no camera startup).

Use the official Orbbec launch file to start cameras first, then run this.

Usage:
  # Terminal 1: Start cameras
  ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py

  # Terminal 2: Start analysis
  ros2 launch ros2_sync_tool sync_analyzer.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    pkg_dir = get_package_share_directory("ros2_sync_tool")
    config_path = os.path.join(pkg_dir, "config", "cameras.yaml")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    analyzer_conf = config.get("analyzer", {})

    analyzer_params = {
        "camera_names": analyzer_conf.get("camera_names", []),
        "stream_types": analyzer_conf.get("stream_types", ["depth", "color"]),
        "duration_sec": analyzer_conf.get("duration_sec", 30),
        "hw_threshold_us": analyzer_conf.get("hw_threshold_us", 500),
        "csv_path": analyzer_conf.get("csv_path", ""),
    }

    analyzer_node = Node(
        package="ros2_sync_tool",
        executable="sync_analyzer_node",
        name="sync_analyzer_node",
        parameters=[analyzer_params],
        output="screen",
    )

    return LaunchDescription([analyzer_node])
```

---

### Task 4: 精简 config/cameras.yaml

**修改文件:**
- `config/cameras.yaml`

**接口:**
- 产出: YAML 配置文件，被 `sync_analyzer.launch.py` 读取

- [ ] **将 `config/cameras.yaml` 整个文件替换为：**

```yaml
# 同步分析器配置
# ---------------------------
# 此文件由 sync_analyzer.launch.py 读取。
#
# 重要：camera_names 必须与官方 launch 文件中的 camera_name 保持一致
# （如 multi_gmsl_camera_synced.launch.py）。

analyzer:
  # 相机名称 — 必须与官方 launch 文件一致
  camera_names:
    - "camera_01"
    - "camera_02"

  # 订阅的流类型（depth, color）
  # Topic 模式: /<camera_name>/<stream_type>/image_raw
  stream_types:
    - "depth"
    - "color"

  # 采集时长（秒）
  duration_sec: 30

  # 硬件时间戳匹配阈值（微秒）
  # 两帧硬件时间戳差值 < 此阈值视为匹配
  hw_threshold_us: 500

  # CSV 导出路径，留空则不导出
  csv_path: ""
```

---

### Task 5: 更新 package.xml

**修改文件:**
- `package.xml`

- [ ] **删除或注释掉 `orbbec_camera` 依赖行**

将：

```xml
  <depend>orbbec_camera</depend>
```

改为注释（分析节点不 link orbbec_camera 库，只订阅标准 sensor_msgs topic）：

```xml
  <!-- orbbec_camera 是运行时依赖（topic 类型用 sensor_msgs 标准消息），非编译依赖 -->
  <!-- <depend>orbbec_camera</depend> -->
```

---

### Task 6: 添加 README.md

**新建文件:**
- `README.md`

- [ ] **创建 `README.md`**

````markdown
# ros2_sync_tool

基于 OrbbecSDK ROS2 的多相机时间戳同步分析工具。

## 功能

订阅多个相机的 `/camera_XX/depth/image_raw` 和 `/camera_XX/color/image_raw` topic，
采集帧时间戳，进行三组对比分析：

1. **跨流对比** — 同一相机 Depth vs Color 时间差
2. **跨设备 Depth 对比** — 不同相机间 Depth 时间差
3. **跨设备 Color 对比** — 不同相机间 Color 时间差

输出硬件时间戳和系统到达时间差异的统计报告（最小值/最大值/均值/标准差），
可选导出 CSV。

## 前置条件

- ROS2（Humble 或更新版本）
- OrbbecSDK_ROS2 已安装并构建
- GMSL 或 USB 相机已连接并配置硬件同步

## 使用方法

### 第一步：启动相机

使用官方 Orbbec launch 文件。GMSL 相机需修改
`OrbbecSDK_ROS2/orbbec_camera/examples/gmsl_camera/multi_gmsl_camera_synced.launch.py`，
为每台相机填入 `usb_port`、`camera_name`、`device_num`。

```bash
# 查看相机端口
ros2 run orbbec_camera list_devices_node

# 启动相机
ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py
```

### 第二步：运行分析

编辑 `config/cameras.yaml`，将 `camera_names` 设为与官方 launch 文件一致的名称。

```bash
ros2 launch ros2_sync_tool sync_analyzer.launch.py
```

分析节点采集 `duration_sec` 秒后自动输出报告。

## 配置参数

见 `config/cameras.yaml`。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `camera_names` | `["camera_01", "camera_02"]` | 必须与官方 launch 名称一致 |
| `stream_types` | `["depth", "color"]` | 订阅的流类型 |
| `duration_sec` | `30` | 采集时长 |
| `hw_threshold_us` | `500` | 硬件时间戳匹配阈值 |
| `csv_path` | `""` | 设为 `"result.csv"` 可导出 CSV |
````

---

### 最终验证

全部修改完成后，确认文件结构：

```bash
ls launch/        # 有 sync_analyzer.launch.py，没有 multi_camera_sync.launch.py
ls config/        # cameras.yaml 已精简
```
