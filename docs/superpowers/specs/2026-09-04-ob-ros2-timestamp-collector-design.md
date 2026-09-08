# OB ROS2 图像与时间戳采集工具设计

> 日期：2026-09-04  
> 状态：待审阅

## 目标

在 `OB ROS2/` 下新建一个独立的 ROS 2 C++ 包。该包只订阅已经由 Orbbec ROS 2 驱动发布的图像话题，采集图像、ROS 时间戳和到达时间，并写出原始 CSV；本版本不做跨相机帧匹配、同步误差统计或 GPIO/PWM 触发控制。

## 运行边界

相机由 Orbbec 官方 launch 文件或用户现有启动方式单独管理。采集工具不启动、不配置 `orbbec_camera` 节点，也不依赖其 C++ API；它仅依赖标准 `sensor_msgs/msg/Image` 话题。

为让 `header.stamp` 可跨设备比较，Orbbec 相机驱动应使用以下配置：

```text
sync_mode:=hardware_triggering
time_domain:=global
enable_sync_host_time:=false
frames_per_trigger:=1
depth_delay_us:=0
color_delay_us:=0
trigger2image_delay_us:=0
```

外部硬件触发本身仍由已有的同步器或 GPIO 触发器提供。共享外部触发拓扑下，通常不应让所有相机都启用 `trigger_out_enabled`。

## 包结构

```text
OB ROS2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── collector.yaml
├── launch/
│   └── timestamp_collector.launch.py
└── src/
    └── timestamp_collector_node.cpp
```

包名为 `ob_ros2_timestamp_collector`。所有文件保持在该目录内，不修改现有 `src/` 直连 Orbbec SDK 工具。

## 节点与配置

`timestamp_collector_node` 通过 YAML 参数接收一组显式订阅定义，每一项包含：

- `camera_name`：用于日志、CSV 和输出路径的稳定相机标识；
- `stream_type`：`color` 或 `depth`；
- `topic`：实际订阅的 `sensor_msgs/msg/Image` 话题。

示例使用 `/camera_01/color/image_raw`、`/camera_01/depth/image_raw`、`/camera_02/color/image_raw` 和 `/camera_02/depth/image_raw`。显式 topic 列表避免假设 Orbbec 的命名空间结构，也允许订阅用户已 remap 的话题。

全局参数：

- `output_dir`：输出根目录，默认 `/tmp/ob_ros2_capture`；
- `duration_sec`：自动停止的采集时长，`0` 表示持续运行直到 ROS 关闭；
- `save_images`：是否保存图像，默认 `false`；
- `max_frames_per_stream`：单流最大记录帧数，`0` 表示不设上限；
- `qos_depth`：订阅队列深度；
- `use_sensor_data_qos`：默认 `true`，使用传感器数据 QoS；
- `flush_every_n_records`：每隔多少条记录刷新 CSV，默认 1，以减少异常退出时的数据丢失。

## 数据流

1. Orbbec ROS 2 驱动发布 `sensor_msgs/msg/Image`。
2. 每个订阅回调记录该消息的 `header.stamp`，换算为微秒；这应为驱动设为 `time_domain=global` 时的全局时间域。
3. 回调同时调用节点时钟获取接收时间，作为 ROS 传输与调度观察值；它不等同于相机硬件时间。
4. 节点生成每个订阅流独立递增的 `frame_index`，写入原始 CSV。
5. 若启用 `save_images`，节点按相机和流类型分别写入图像文件，并让 CSV 行引用相对路径。
6. 到达采集时长或帧数上限后，节点停止各订阅，刷新并关闭 CSV，在控制台输出每个流的帧数和时间范围，然后退出。

## 输出格式

输出根目录内创建一次采集目录，包含：

```text
<output_dir>/capture_YYYYmmdd_HHMMSS/
├── timestamps.csv
└── images/                         # 仅 save_images=true 时存在
    └── <camera_name>/<stream_type>/
```

`timestamps.csv` 的列固定如下：

```text
camera_name,stream_type,frame_index,header_stamp_us,receive_stamp_us,frame_id,width,height,encoding,step,data_size,image_path
```

- `header_stamp_us`：从 `msg.header.stamp` 得到，用作后续同步分析的候选全局时间戳；
- `receive_stamp_us`：节点收到消息时的时钟值；
- `frame_id`：ROS header 的坐标系标识，不把它误用为帧序号；
- `frame_index`：采集节点生成的每流顺序号；
- `image_path`：未保存图像时为空。

图像保存不引入 OpenCV：彩色和深度消息都以其原始 ROS `Image` 字节布局写成 `.bin`，并由 CSV 保留 `encoding`、`width`、`height`、`step` 和 `data_size` 以便无损复原。这样不会将 `YUYV`、`RGB8`、`16UC1` 等编码错误转换成 PNG。后续若用户需要浏览友好的 PNG，可增加单独的转换程序。

## 错误处理与资源保护

- 空的 `subscriptions` 参数、空 `camera_name`、重复订阅标签或不支持的 `stream_type` 会在启动时报告并终止；
- 输出目录或 CSV 无法创建时节点立即失败，不开始订阅；
- 图像文件写入失败会报告错误，并继续记录时间戳，CSV 中该行的 `image_path` 留空；
- 为避免慢速磁盘 I/O 阻塞相机回调，初版仅在 `save_images=true` 时将消息复制到有上限的后台写盘队列；队列满时丢弃该图像保存任务、保留时间戳 CSV，并在日志中累计报告保存丢弃数；
- CSV 写入由互斥锁保护；统计计数与帧序号按订阅流分别维护。

## 验证

1. 在 ROS 2 Humble 或兼容发行版的工作空间中执行 `colcon build --packages-select ob_ros2_timestamp_collector`。
2. 使用 `ros2 topic pub` 或现有图像话题验证单话题订阅、CSV 列值和自动停止。
3. 在两台已启用 `time_domain=global` 的 Orbbec 相机上验证四个话题同时写入，每个流帧数可在退出报告和 CSV 中一致核对。
4. 使用 `save_images=true` 验证每条成功保存图像的 CSV 行都有可读的 `.bin` 文件，且文件大小与 `data_size` 一致。

## 非目标

- 不对帧进行时间戳配对；
- 不输出同步偏差或同步质量结论；
- 不启动或配置 Orbbec 相机驱动；
- 不写入 `/sys/kernel/debug/gpio_trigger/framerate` 或控制任何硬件触发线路；
- 不改变现有 `src/` SDK 工具。
