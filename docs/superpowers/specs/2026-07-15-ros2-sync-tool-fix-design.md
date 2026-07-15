# ros2_sync_tool 修复设计

## Context

`ros2_sync_tool` 是一个多相机硬件同步精度分析工具。它订阅多个 Orbbec 相机的 ROS2 image topic，收集帧时间戳，然后做三组统计分析（跨流/跨设备 Depth/跨设备 Color）。

当前问题：
1. `hwTimestampUs` 和 `sysTimestampUs` 取同一个值（`header.stamp`），硬件同步 vs 系统到达的对比无意义
2. launch 文件把相机启动和分析混在一起，不符合官方最佳实践
3. GMSL 同步参数缺失

## 架构：相机与分析解耦

```
┌────────────────────────────────┐     ┌────────────────────────────────┐
│  Orbbec 官方 launch (启动相机)    │     │  ros2_sync_tool (分析)          │
│                                │     │                                │
│  ros2 launch orbbec_camera     │     │  ros2 launch ros2_sync_tool    │
│  multi_gmsl_camera_synced      │     │  sync_analyzer.launch.py       │
│  .launch.py                    │     │                                │
│              ↓                 │     │  订阅 topic:                    │
│  /camera_01/depth/image_raw ───┼─────▶  /camera_01/depth/image_raw    │
│  /camera_01/color/image_raw ───┼─────▶  /camera_01/color/image_raw    │
│  /camera_02/depth/image_raw ───┼─────▶  /camera_02/depth/image_raw    │
│  /camera_02/color/image_raw ───┼─────▶  /camera_02/color/image_raw    │
│                                │     │              ↓                 │
│                                │     │  SyncAnalyzerCore 分析         │
│                                │     │              ↓                 │
│                                │     │  终端报告 + 可选 CSV            │
└────────────────────────────────┘     └────────────────────────────────┘
```

- 相机启动由官方 launch 文件负责（`multi_gmsl_camera_synced.launch.py`）
- `ros2_sync_tool` 只负责分析，不启动相机
- 两者通过 `camera_name` 关联（必须一致）

## 时间戳来源

订阅 `sensor_msgs/msg/Image`（`/camera_XX/color/image_raw` 和 `/camera_XX/depth/image_raw`）：

- **硬件时间戳**：`msg->header.stamp`，官方默认 `time_domain="global"`，值为 `frame->getGlobalTimeStampUs()`
- **系统到达时间**：`this->now()`，记录 ROS 节点收到消息的时间

## 改动文件

### 1. `src/ros2_sync_analyzer_node.cpp` — 修复时间戳

在 `imageCallback` 中，将 `sysTimestampUs` 从 `header.stamp` 改为 `this->now()`：

```
// 改前
fs.sysTimestampUs = fs.hwTimestampUs;  // 同一来源

// 改后
fs.sysTimestampUs = this->now().nanoseconds() / 1000LL;  // ROS 回调时间
```

### 2. `launch/multi_camera_sync.launch.py` — 删除

相机启动不再由此工具负责，移交给官方 launch 文件。

### 3. `launch/sync_analyzer.launch.py` — 新建

只启动分析节点：

```python
# 功能：启动 sync_analyzer_node，参数从 cameras.yaml 读取
# 用法：ros2 launch ros2_sync_tool sync_analyzer.launch.py
```

### 4. `config/cameras.yaml` — 精简

删除相机硬件参数（`serial_number`、`sync_mode`、`streams` 分辨率），只保留分析配置：

```yaml
# 分析器配置
# 注意：camera_names 需要与官方 launch 文件中的 camera_name 一致
analyzer:
  camera_names:
    - "camera_01"
    - "camera_02"
  stream_types:
    - "depth"
    - "color"
  duration_sec: 30
  hw_threshold_us: 500
  csv_path: ""
```

### 5. `README.md` — 更新使用说明

### 6. `package.xml` — 移除 `orbbec_camera` depend

分析节点不依赖 `orbbec_camera`，改为可选的 `exec_depend`。

### 不改的文件

- `src/sync_analyzer_core.cpp/h`
- `src/frame_stamp.h`
- `CMakeLists.txt`

## 官方 launch 文件配置（OrbbecSDK_ROS2 侧）

以下两个文件在 `OrbbecSDK_ROS2/orbbec_camera/` 目录下，首次使用需要修改，之后不变。

### `examples/gmsl_camera/multi_gmsl_camera_synced.launch.py`

这是 GMSL 多相机同步的启动入口。需要修改的地方：

```python
# ① 相机数量：有几个相机，就加几个 launch_include 块
#    每个块设置：camera_name、usb_port

# ② 查 usb_port（所有相机插好后）
ros2 run orbbec_camera list_devices_node
# 输出类似：gmsl2-1, gmsl2-3, ...

# ③ 修改每个相机块的参数
launch1_include = IncludeLaunchDescription(
    ..."gemini_330_gmsl.launch.py"),
    launch_arguments={
        "camera_name": "camera_01",         # 第1个相机
        "usb_port": "gmsl2-1",              # 从 list_devices_node 查到
        "device_num": "2",                  # 相机总数
        "sync_mode": "secondary_synced",    # GMSL 全部用 secondary_synced
        "gmsl_trigger_fps": "3000",         # 硬件触发帧率
        "enable_gmsl_trigger": "true",      # 开启 GMSL 硬件触发
        ...
    }.items(),
)

launch2_include = IncludeLaunchDescription(
    ..."gemini_330_gmsl.launch.py"),
    launch_arguments={
        "camera_name": "camera_02",         # 第2个相机
        "usb_port": "gmsl2-3",              # 从 list_devices_node 查到
        "device_num": "2",                  # 相机总数
        "sync_mode": "secondary_synced",
        ...
    }.items(),
)

# ④ 相机A (launch2_include) 在 0 秒启动，相机B (launch1_include) 在 2 秒后启动
#    注意：相机列表中最后那个是 TimerAction(period=0.0)
```

**关键参数说明**：

| 参数 | 值 | 说明 |
|---|---|---|
| `sync_mode` | `secondary_synced` | GMSL 不需要 primary，全部设此值 |
| `enable_gmsl_trigger` | `true` | 开启 SOC 硬件触发 |
| `gmsl_trigger_fps` | `3000` | 触发帧率（与相机内部帧率一致即可） |
| `camera_name` | `camera_01`, `camera_02`... | 命名规则：`camera_0X` |
| `usb_port` | `gmsl2-1` 等 | 通过 `list_devices_node` 获取 |

### `config/camera_secondary_params.yaml`

相机流配置，所有从相机共用：

```yaml
# 核心参数
time_domain: "global"          # 使用全局时间戳
enable_sync_host_time: false   # global 域下必须为 false

# 流开关
enable_color: true
enable_depth: true
enable_point_cloud: false      # 不需要点云
enable_colored_point_cloud: false

# 流分辨率（0 = 跟随设备预设）
color_width: 0
color_height: 0
color_fps: 0
depth_width: 0
depth_height: 0
depth_fps: 0
```

## 完整工作流程

### Step 1：硬件准备

```
相机A ── GMSL线 ──┐
相机B ── GMSL线 ──├── 主板 ── 电脑
```

所有 GMSL 相机通过 GMSL 线连接到主板，主板通过 USB/PCIe 连接电脑。

### Step 2：查相机端口

```bash
ros2 run orbbec_camera list_devices_node
# 记录每个相机的 usb_port（如 gmsl2-1, gmsl2-3）
```

### Step 3：修改官方 launch 文件（首次配置，之后不改）

- `examples/gmsl_camera/multi_gmsl_camera_synced.launch.py` → 填入 `camera_name`、`usb_port`、`device_num`
- `config/camera_secondary_params.yaml` → 确认 `time_domain: "global"`、`enable_sync_host_time: false`

### Step 4：修改分析配置

`ros2_sync_tool/config/cameras.yaml` → 填入与 Step 3 相同的 `camera_names`

### Step 5：启动

```bash
# 终端1：启动相机
ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py

# 终端2：启动分析（等待相机稳定输出后再启动）
ros2 launch ros2_sync_tool sync_analyzer.launch.py
```

### Step 6：查看结果

分析节点运行 `duration_sec` 秒后自动输出报告并退出。报告示例：

```
==============================================
  Timestamp Sync Analysis Report
==============================================
--- 1. Cross-Stream (Depth vs Color) ---
Device 0:
  Pair count: 897
  HW Timestamp Diff:
    Min=-320us  Max=280us  Mean=15.2us  Stddev=85.3us
  System Timestamp Diff:
    Min=-1500us  Max=1200us  Mean=450.8us  Stddev=310.5us
...
```

## 验证

1. 启动官方 GMSL 同步 launch
2. `ros2 launch ros2_sync_tool sync_analyzer.launch.py`
3. 等待采集结束，检查报告：
   - `sysTimestampUs` ≠ `hwTimestampUs`
   - "HW Timestamp Diff" 均值应较小（硬件已同步，通常 < 500us）
   - "System Timestamp Diff" 应明显大于 HW Diff（反映 ROS 传输延迟差异）
   - 可选 `csv_path: "result.csv"` 导出验证
