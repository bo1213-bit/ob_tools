# ROS2 多相机时间戳同步分析工具 — 设计文档

> 日期: 2026-07-14
> 状态: 设计中

## 1. 概述

### 1.1 背景

现有 `ob_tools` 项目使用 Orbbec C++ SDK (`libobsensor`) 直接控制多台相机进行硬同步，采集 Depth + Color 双流帧数据，对比不同相机/不同流之间的时间戳差异。

目标：新增 ROS2 版本，通过 OrbbecSDK ROS2 Wrapper (`orbbec_camera_node`) 管理相机，编写 ROS2 分析节点订阅话题、采集帧、对比时间戳。

### 1.2 约束

- 相机通过 GMSL 接口连接，使用 `serial_number` 区分相机
- Launch 文件一次性启动所有相机节点 + 分析节点
- 先用 `sensor_msgs/Image` 的 `header.stamp` 作为时间戳（后续可切换到 metadata 话题）
- 独立 ROS2 package，与现有 C++ SDK 代码不冲突
- 提取 `sync_analyzer` 核心逻辑为共享代码，两个前端复用

### 1.3 目标

一个 ROS2 package `ros2_sync_tool`，包含：
- Launch 文件：启动 N 个 `orbbec_camera_node`（每个相机一个 namespace） + 1 个分析节点
- 分析节点：订阅所有相机话题，采集帧，做三类时间戳对比，输出报告 + CSV

---

## 2. Package 结构

```
ob_tools/
├── CMakeLists.txt                  # 现有 C++ SDK 项目（不动）
├── src/
│   ├── data_collector.cpp/h        # 现有 C++ SDK 采集（不动）
│   ├── sync_analyzer.cpp/h         # 现有分析逻辑（不动）
│   ├── frame_stamp.h               # 共享数据结构（不动）
│   └── main.cpp                    # 现有 C++ SDK 入口（不动）
│
└── ros2_sync_tool/                 # 新：ROS2 独立 package（完全自包含）
    ├── package.xml
    ├── CMakeLists.txt
    ├── launch/
    │   └── multi_camera_sync.launch.py    # 启动所有相机 + 分析节点
    ├── config/
    │   └── cameras.yaml                   # 相机 SN 列表 + 流配置
    └── src/
        ├── frame_stamp.h                  # 复制自 ../src/frame_stamp.h（独立副本）
        ├── sync_analyzer_core.cpp/h       # 新建：纯分析逻辑（不依赖 ob::Device）
        └── ros2_sync_analyzer_node.cpp    # ROS2 分析节点
```

---

## 3. 架构与数据流

```
┌─────────────────────────────────────────────────────────┐
│  launch: multi_camera_sync.launch.py                    │
│                                                         │
│  ┌─────────────────────┐  ┌─────────────────────┐      │
│  │ orbbec_camera_node  │  │ orbbec_camera_node  │      │
│  │ namespace: camera_0 │  │ namespace: camera_1 │      │
│  │ SN: "XX001"         │  │ SN: "XX002"         │      │
│  │ sync_mode: primary  │  │ sync_mode: secondary │      │
│  └──────┬──────────────┘  └──────┬──────────────┘      │
│         │ /camera_0/color/image_raw                     │
│         │ /camera_0/depth/image_raw                     │
│         │                        │ /camera_1/color/...   │
│         │                        │ /camera_1/depth/...   │
│         ▼                        ▼                      │
│  ┌──────────────────────────────────────────────┐       │
│  │         sync_analyzer_node                    │       │
│  │                                               │       │
│  │  params: camera_names=["camera_0","camera_1"] │       │
│  │          duration_sec=30                      │       │
│  │          hw_threshold_us=500                  │       │
│  │          csv_path="result.csv"                │       │
│  │                                               │       │
│  │  ┌──────────────┐   ┌───────────────────┐    │       │
│  │  │ TopicManager │   │ AnalyzerCore      │    │       │
│  │  │ - 动态订阅    │──▶│ - matchAndDiff    │    │       │
│  │  │ - 中转存储    │   │ - computeStats    │    │       │
│  │  │ - 计数+超时   │   │ - printReport     │    │       │
│  │  └──────────────┘   │ - exportCSV       │    │       │
│  │                      └───────────────────┘    │       │
│  └──────────────────────────────────────────────┘       │
└─────────────────────────────────────────────────────────┘
```

### 工作流程

1. **Launch** 启动 N 个相机节点（各自 namespace）+ 1 个分析节点
2. **分析节点启动**：读取 `camera_names` 参数，动态构造话题名，创建订阅
3. **回调中**：每收到一帧，提取 `header.stamp` 转为微秒时间戳 → 存入 `FrameStamp`
4. **采集阶段**：运行 `duration_sec` 秒，之后断开订阅
5. **分析阶段**：调用 `AnalyzerCore` 做三类对比，输出控制台报告 + CSV
6. **节点退出**

---

## 4. Launch 文件设计

### 4.1 核心配置

```python
# multi_camera_sync.launch.py
# 用户只需要修改这个列表即可适配不同相机
cameras = [
    {"name": "camera_0", "sn": "XX001", "sync_mode": "primary"},
    {"name": "camera_1", "sn": "XX002", "sync_mode": "secondary"},
]
```

Launch 文件遍历 `cameras` 列表，为每台相机启动一个 `orbbec_camera_node`，然后启动 `sync_analyzer_node`。

### 4.2 相机节点参数

| 参数 | 来源 | 说明 |
|------|------|------|
| `camera_name` | launch 配置 | 相机 namespace |
| `serial_number` | launch 配置 | 相机序列号 |
| `sync_mode` | launch 配置 | primary / secondary |
| `depth_width/height/fps/format` | cameras.yaml | 深度流配置 |
| `color_width/height/fps/format` | cameras.yaml | 彩色流配置 |
| `enable_sync_host_time` | True | 启用主机时间同步 |
| `trigger_out_enabled` | primary 才开 | 触发输出 |
| `frames_per_trigger` | 1 | 每次触发采集 1 帧 |

### 4.3 分析节点参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `camera_names` | string[] | 必填 | 如 `["camera_0", "camera_1"]` |
| `stream_types` | string[] | `["depth", "color"]` | 要订阅的流类型 |
| `duration_sec` | int | 30 | 采集时长 |
| `hw_threshold_us` | int | 500 | 时间戳配对阈值 |
| `csv_path` | string | `""` | CSV 导出路径（空=不导出） |

---

## 5. 话题订阅设计

### 5.1 话题名构造

分析节点根据 `camera_names` × `stream_types` 拼出话题名：

```
/camera_0/depth/image_raw
/camera_0/color/image_raw
/camera_1/depth/image_raw
/camera_1/color/image_raw
```

### 5.2 回调逻辑

每收到一帧 `sensor_msgs::msg::Image`：

1. 从话题名解析 `camera_name` → 映射到 camera_index
2. 从话题名解析 `stream_type` → 映射到 stream_index
3. 提取 `header.stamp`：`timestamp_us = sec * 1_000_000 + nanosec / 1_000`
4. 构造 `FrameStamp`，push_back 到 `allFrames_[camera_index][stream_index]`

### 5.3 采集终止

启动一个 ROS2 timer（`duration_sec` 秒后触发），回调中：
- 关闭所有 subscription
- 调用 `SyncAnalyzerCore::run()`
- 调用 `printReport()` + `exportCSV()`
- 调用 `rclcpp::shutdown()`

---

## 6. 分析逻辑复用

### 6.1 策略：独立副本，不改原代码

不修改现有 `src/sync_analyzer.cpp/h`。ROS2 package 内部新建 `sync_analyzer_core.cpp/h`，从现有 `sync_analyzer.cpp` **提取**纯计算逻辑（`matchAndDiff`、`computeStats`、`printReport`、`exportCSV`），去掉 `ob::Device` 依赖。两个文件独立存在，互不干扰。

### 6.2 sync_analyzer_core

```cpp
// sync_analyzer_core.h — 纯数据计算，不依赖任何 SDK
class SyncAnalyzerCore {
public:
    struct Config {
        int64_t hwThresholdUs = 500;
    };

    // 输入：三维 FrameStamp 数组 [device][stream][frame]
    // 输出：三类对比统计 + 原始 diff
    void run(const std::vector<std::vector<std::vector<FrameStamp>>>& frames,
             const Config& cfg);

    const std::vector<PairStats>& getCrossStreamStats() const;
    const std::vector<PairStats>& getCrossDeviceDepthStats() const;
    const std::vector<PairStats>& getCrossDeviceColorStats() const;
    void printReport() const;
    void exportCSV(const std::string& path) const;

private:
    static std::pair<std::vector<int64_t>, std::vector<int64_t>>
    matchAndDiff(const std::vector<FrameStamp>& a,
                 const std::vector<FrameStamp>& b,
                 int64_t hwThresholdUs);

    static PairStats computeStats(int devI, int devJ, StreamType st,
                                  bool isCrossStream,
                                  const std::vector<int64_t>& hwDiffs,
                                  const std::vector<int64_t>& sysDiffs);

    std::vector<PairStats> crossStreamStats_;
    std::vector<PairStats> crossDeviceDepthStats_;
    std::vector<PairStats> crossDeviceColorStats_;
    std::vector<DiffRecord> allDiffs_;
};
```

`sync_analyzer_core.cpp` 的实现从现有 `sync_analyzer.cpp` 搬过来，类名和命名空间调整即可，核心算法一模一样。

---

## 7. FrameStamp 适配说明

`FrameStamp` 结构体保持不变：

```cpp
struct FrameStamp {
    int64_t    hwTimestampUs;   // 硬件时间戳
    int64_t    sysTimestampUs;  // 软件/系统时间戳
    int64_t    frameNumber;     // 帧序号
    int        deviceIndex;     // 设备索引
    StreamType streamType;      // DEPTH 或 COLOR
};
```

ROS2 版本中时间戳映射：

| 字段 | 来源 | 说明 |
|------|------|------|
| `hwTimestampUs` | `header.stamp` | 先用 ROS 时间戳，后续可切换到 metadata 话题 |
| `sysTimestampUs` | `header.stamp` | 同上，此时 hw 和 sys 相同 |
| `frameNumber` | `header.stamp` 转换 | 或者用 `header.frame_id` 中的序列号 |
| `deviceIndex` | camera_name 映射 | 0, 1, 2... |
| `streamType` | 话题名解析 | depth=0, color=1 |

> 后续升级：当 metadata 话题的消息类型确认后，将 `hwTimestampUs` 改为从 metadata 话题获取硬件时间戳，`sysTimestampUs` 保持 `header.stamp`。

---

## 8. CMake 构建

### 8.1 ros2_sync_tool/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(ros2_sync_tool)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 分析核心库
add_library(sync_analyzer_core
    src/sync_analyzer_core.cpp
)
target_include_directories(sync_analyzer_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ROS2 分析节点
add_executable(sync_analyzer_node
    src/ros2_sync_analyzer_node.cpp
)
target_link_libraries(sync_analyzer_node sync_analyzer_core)
ament_target_dependencies(sync_analyzer_node rclcpp sensor_msgs)
target_include_directories(sync_analyzer_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

install(TARGETS sync_analyzer_node sync_analyzer_core
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 8.2 现有 CMakeLists.txt

**不动。** 现有 C++ SDK 项目保持不变，ROS2 package 独立构建，两个项目完全解耦。

---

## 9. dependencies

### 9.1 package.xml

```xml
<package format="3">
  <name>ros2_sync_tool</name>
  <version>0.1.0</version>
  <description>Multi-camera timestamp sync analysis tool using OrbbecSDK ROS2</description>
  <maintainer email="leju@example.com">leju</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>orbbec_camera</depend>  <!-- 提供 orbbec_camera_node -->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 10. 测试验证

- 与现有 C++ SDK 工具对比，相同参数下三类对比结果应一致（在时间戳精度允许范围内）
- 单相机场景：至少能跑通，跨设备对比为空
- 多相机场景：跨设备 Depth/Color 对比有数据
- CSV 输出格式与现有工具一致，可视化脚本可直接复用

---

## 11. 文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `ros2_sync_tool/package.xml` | 新建 | ROS2 package 描述 |
| `ros2_sync_tool/CMakeLists.txt` | 新建 | ROS2 构建脚本 |
| `ros2_sync_tool/launch/multi_camera_sync.launch.py` | 新建 | Launch 文件 |
| `ros2_sync_tool/config/cameras.yaml` | 新建 | 相机配置 |
| `ros2_sync_tool/src/frame_stamp.h` | 新建 | 复制自 `src/frame_stamp.h`（独立副本） |
| `ros2_sync_tool/src/sync_analyzer_core.h` | 新建 | 分析逻辑头文件（不依赖 SDK） |
| `ros2_sync_tool/src/sync_analyzer_core.cpp` | 新建 | 分析逻辑实现 |
| `ros2_sync_tool/src/ros2_sync_analyzer_node.cpp` | 新建 | ROS2 分析节点 |
| `src/*` | **不动** | 现有 C++ SDK 代码全部保持不变 |
| `CMakeLists.txt` | **不动** | 现有构建脚本不变 |