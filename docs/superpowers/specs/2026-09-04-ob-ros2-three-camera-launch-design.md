# OB ROS2 三相机外部触发启动设计

> 日期：2026-09-04  
> 状态：待审阅

## 目标

扩展已有的 `ob_ros2_timestamp_collector` ROS 2 Jazzy 包，使它能以项目自有 launch 同时启动三台外部硬件触发的 GMSL 相机，并可与已有时间戳采集器组合启动。

目标设备及命名空间如下：

| ROS 命名空间 | 型号 | 序列号 | GMSL 端口 |
| --- | --- | --- | --- |
| `camera_305_01` | Gemini 305g | `CV3T561000B5` | `gmsl2-5` |
| `camera_305_02` | Gemini 305g | `CV3T561000H0` | `gmsl2-4` |
| `camera_335lg_01` | Gemini 335Lg | `CPBG1630011D` | `gmsl2-6` |

## 已验证的 Jazzy 驱动契约

目标环境为 Ubuntu 24.04、ROS 2 Jazzy、Orbbec ROS Wrapper/SDK 2.7.6。

- 305g 已用官方文件 `/opt/ros/jazzy/share/orbbec_camera/examples/gmsl_camera/gemini_330_gmsl.launch.py` 成功启动。该 launch 支持 `config_file_path`、`enable_gmsl_trigger`、`serial_number`、`sync_mode`、`frames_per_trigger`、`trigger_out_enabled`、`enable_point_cloud`、`enable_sync_host_time` 和 `time_domain`。
- 335Lg 已用 `/opt/ros/jazzy/share/orbbec_camera/launch/gemini345_lg.launch.py` 成功启动并发布 color 数据。它支持除 `enable_gmsl_trigger` 外的相同核心参数；因此不得向 335Lg 传入该参数。
- 335Lg 在共享外部触发下实际 color 输出为约 15 Hz，这是已知硬件行为。它默认启动 profile 的日志为 Color `1280x720@30 YUYV`、Depth `1280x720@30 Y16`，但项目配置不固定这些 profile。
- 335Lg 的官方 launch 默认 `device_preset` 为 `Standard`，该设备不接受此值。项目 YAML 必须明确使用 `device_preset: Default`。

## 触发和时间边界

外部 Jetson GPIO 已负责产生共享触发。项目 launch：

- 不写入 GPIO sysfs；
- 不生成 GMSL/SoC trigger；
- 不开启任何相机 trigger output；
- 仅将相机配置为被动的 `hardware_triggering` 模式。

三台设备的共同参数为：

```yaml
sync_mode: hardware_triggering
frames_per_trigger: 1
trigger_out_enabled: false
enable_point_cloud: false
enable_sync_host_time: false
time_domain: global
enable_color: true
enable_depth: true
device_preset: Default
```

两台 305g 额外设置 `enable_gmsl_trigger: false`。335Lg YAML 不包含该参数。

## 包结构与组件

新增文件：

```text
OB ROS2/
├── config/gmsl/
│   ├── camera_305_01.yaml
│   ├── camera_305_02.yaml
│   └── camera_335lg_01.yaml
└── launch/
    └── three_gmsl_external_trigger.launch.py
```

### 每相机 YAML

每份 YAML 仅包含一台设备的稳定绑定和已验证的同步参数。它不声明 color/depth 的宽高、格式或 FPS，让实际设备和 Jazzy 驱动选择其默认兼容 profile。这避免在未完成三机联合测试前将未经验证的 profile 写入配置。

### 三相机 launch

`three_gmsl_external_trigger.launch.py` 直接启动三组独立的组件容器；每个容器载入一个 `orbbec_camera::OBCameraNodeDriver`：

- 每台相机有独立的 namespace、component node 名称和 container 名称；
- 每个组件从其对应 YAML 文件加载参数；
- 不通过 `IncludeLaunchDescription` 三次嵌套官方 launch。官方 305g GMSL launch 每次会创建同名共享容器，多次 include 会引入重名冲突；独立容器消除了该风险；
- launch 只依赖已安装的 `orbbec_camera` 和 `rclcpp_components` 运行时组件。

节点预期发布的输入话题为：

```text
/camera_305_01/color/image_raw
/camera_305_01/depth/image_raw
/camera_305_02/color/image_raw
/camera_305_02/depth/image_raw
/camera_335lg_01/color/image_raw
/camera_335lg_01/depth/image_raw
```

首次三机启动后必须通过 `ros2 topic list` 复核这些主题；若安装驱动版本的真实名称不同，应只修正 collector YAML 的显式订阅路径。

## 与采集器的集成

更新 `config/gmsl_scheme_a_collector.yaml` 为上述三台相机的六个 color/depth 订阅项。采集器 C++ 源码不作修改。

更新 `launch/gmsl_scheme_a_capture.launch.py`，默认包含项目内的 `three_gmsl_external_trigger.launch.py` 并启动 `timestamp_collector_node`。组合启动保留 `collector_config_file` 覆盖参数。

## 错误处理和可维护性

- 每台相机用序列号绑定而非枚举顺序，避免 GMSL 设备发现顺序变化导致误绑定。
- 相机进程启动失败时 ROS launch 需要保留该相机的命名空间、容器和驱动错误日志，便于定位单台设备问题。
- launch 不承担相机间帧配对、同步精度计算或硬件触发控制职责。
- `timestamp_collector_node` 的 CSV 与原始图像持久化行为保持不变。

## 验证流程

1. 启动外部 GPIO 30 Hz 触发（项目 ROS launch 不执行此操作）。
2. 在 Jetson 的 ROS 2 工作空间执行 `colcon build --packages-select ob_ros2_timestamp_collector` 并 `source install/setup.bash`。
3. 启动 `three_gmsl_external_trigger.launch.py`，用 `ros2 node list` 确认三台独立节点，用 `ros2 topic list` 确认六个预期 image topic。
4. 分别使用 `ros2 topic hz` 验证各 color/depth 流持续发布；335Lg color 约 15 Hz 是当前拓扑下预期值。
5. 启动组合 capture launch，先设置 `duration_sec: 10`、`save_images: false`，确认 CSV 包含全部六流记录。
6. 设为 `save_images: true`，验证所有非空 `image_path` 文件存在且文件大小与对应 `data_size` 一致，并确认退出日志没有 `dropped_image_jobs`。

## 非目标

- 不修改 `D:\Data\robotPackage\ob_tools\src` 下的直接 Orbbec SDK 程序；
- 不写 GPIO 或产生触发；
- 不在采集器中实施跨相机配对或同步精度计算；
- 不锁定未经过三机联合验证的色彩/深度 profile；
- 不修改、安装或复制 `/opt/ros/jazzy/share/orbbec_camera` 中的官方驱动文件。
