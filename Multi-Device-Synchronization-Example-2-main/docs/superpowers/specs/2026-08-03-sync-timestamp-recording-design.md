# MultiDeviceSync 图像+时间戳录制

## Context

验证多设备（Orbbec 相机）时间戳同步精度。当前 `MultiDeviceSync.cpp` 只能实时预览，没有持久化功能。
用户需要：相机拍秒表，保存图像和时间戳，离线分析同步精度。设备运行在 Secondary-Synced 模式。

## 命令行接口

```
MultiDeviceSync --record <seconds>    # 录制 N 秒后自动停止
MultiDeviceSync                        # 原有行为，只显示不保存
```

## 保存内容

- **只保存彩色帧**（用户只需拍秒表验证，不需深度数据）
- 图像为窗口里看到的样子（复用 `utils_opencv.cpp` 中 `visualize()` 的可视化结果）
- 时间戳只记录 `deviceTimestampUs`（设备硬件时钟域）

## 输出结构

```
./sync_capture_YYYYMMDD_HHMMSS/
├── timestamps.csv
├── Device0_frame_000001_<timestamp>.png
├── Device0_frame_000002_<timestamp>.png
├── Device1_frame_000001_<timestamp>.png
├── Device1_frame_000002_<timestamp>.png
└── ...
```

## CSV 格式

```
groupId,deviceIndex,deviceSN,deviceTimestampUs,fileName
```

## 录制控制

- 以 `steady_clock::now()` 为基准，每轮检查 `elapsed < recordSeconds`
- 时间到后自动停止录制，关闭 CSV，打印统计信息
- 不限制帧数上限

## 文件命名

`Device{index}_frame_{递增序号6位}_{deviceTimestampUs}.png`

## 改动文件

新建 `MultiDeviceSync/MultiDeviceSyncRecord.cpp`，不修改原有 `MultiDeviceSync.cpp`。

复用 `common/` 目录中的 `PipelineHolder`、`FramePairingManager`、`utils_opencv` 等公共模块。
从 `MultiDeviceSync.cpp` 复制必要的脚手架（配置加载、设备同步设置、开流逻辑），将显示循环替换为录制循环。

## 验证

1. 运行 `MultiDeviceSync --record 10` 录制10秒
2. 检查 `./sync_capture_*` 目录下 PNG 文件和 CSV 是否生成
3. 随机打开几张图，确认内容和时间戳对应
4. 分析 CSV 中多设备 timestamp 差值是否在预期精度内
