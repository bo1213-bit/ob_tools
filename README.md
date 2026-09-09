# 多相机时间戳同步校验工具（ob_tools）

基于 Orbbec SDK 的多相机硬件触发时间戳同步校验程序。

- **硬件**：3 台 Orbbec 相机（2× Gemini 305g + 1× Gemini 335Lg），GMSL2 / FAKRA 接口
- **触发模式**：`OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING`
- **目标**：在同一外部触发沿下，使用全局时间戳评估多相机同步精度

完整的同步配置、开流、采集、诊断和匹配说明见：[同步配置、开流与全局时间戳匹配流程](docs/synchronization-flow.md)。

---

## 一、处理流程

入口 `src/main.cpp` 按以下顺序运行：

```text
DataCollector（采集） → 可选 raw CSV 导出 → SyncAnalyzer（匹配与统计） → 报告与结果 CSV
```

### 1. 设备配置与全局时钟

`DataCollector::run()` 的主要步骤：

1. 枚举设备，输出 SN、型号、PID、固件及 global timestamp 支持情况；
2. 写入硬件触发同步配置；
3. 启用全局时间戳并同步设备时钟；
4. 创建并启动各设备 Pipeline；
5. 打开共享录制 gate，采集指定时长；
6. 冻结录制、停止 Pipeline，最后才关闭自动触发；
7. 对采集帧进行 global timestamp 匹配和统计。

硬件同步配置必须保持为：

```cpp
cfg.syncMode             = OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING;
cfg.triggerOutEnable     = true;
cfg.depthDelayUs         = 0;
cfg.colorDelayUs         = 0;
cfg.trigger2ImageDelayUs = 0;
cfg.triggerOutDelayUs    = 0;
cfg.framesPerTrigger     = 1;
```

> `triggerOutEnable` 是当前硬件同步要求的一部分，必须保持 `true`，不要作为普通排障手段关闭。

所有支持 global timestamp 的设备必须按以下官方兼容顺序初始化：

```text
enableGlobalTimestamp(true)（全部设备）
→ Context::enableDeviceClockSync(0)
→ 等待 1 秒稳定
```

335Lg 在当前部署中还必须在设备支持且可写时启用 `OB_PROP_FPS_BOOST_BOOL=true`；否则在 30 Hz 触发下可能只输出约一半帧率。程序会设置并读回该属性状态。

### 2. 开流与采集窗口

当前请求的流 profile 为：

- Depth：`OB_STREAM_DEPTH`、`OB_FORMAT_Y16`
- Color：`OB_STREAM_COLOR`、`OB_FORMAT_YUYV`
- 默认配置：`1280x800 @ 30 fps`

Pipeline 当前按设备枚举顺序确定性地依次 `start()`：

```cpp
for (int camIndex = 0; camIndex < deviceCount; ++camIndex) {
    pipelines_[camIndex]->start(streamCfgs[camIndex], callback);
}
```

这替代了此前的并发屏障启动方式，但它是用于验证“启动顺序是否影响残余 global 相位关系”的**受控实验**，不是已经证实的同步修复。进行板端对比时应只改变这一项，避免同时修改 profile、FPS boost、触发生命周期或匹配规则。

Pipeline 启动过程中 `recordingEnabled_` 保持关闭，因此启动早期回调不会进入最终帧列表或 raw CSV。所有 Pipeline 启动完成后才打开共享正式采集窗口。

当使用 `--trigger-hz=N`（`N > 0`）时，程序会：

```text
写 0 停止自动触发 → 启动全部 Pipeline → 等待约 300 ms → 写 N 启动触发
```

Color 流可进行预热，预热数据会在正式录制前清空。该 gate 控制的是回调是否接受，不能单独证明帧的曝光时刻；仍应结合帧序号、时间戳和诊断信息判断。

安全停流顺序必须是：

```text
冻结 recording / saving gate
→ 保持触发运行并停止全部 Pipeline
→ 最后才写 0 关闭自动触发
```

---

## 二、时间戳匹配与同步精度

每帧保存的时间戳字段如下：

| 字段 | SDK API | 用途 |
|---|---|---|
| `hwTimestampUs` | `frame->timeStampUs()` | 设备硬件时间戳，仅作诊断/对照 |
| `globalTimestampUs` | `frame->globalTimeStampUs()` | **帧匹配与同步精度的主时间戳** |
| `sysTimestampUs` | `frame->systemTimeStampUs()` | 主机接收和回调诊断 |
| `frameNumber` | `frame->getIndex()` | 帧连续性诊断 |

分析器按 `globalTimestampUs` 排序，使用约半帧周期的配对容差：

```text
matchTolUs = round(1,000,000 / fps / 2)
```

30 fps 时约为 `16667 us`。该容差只用于寻找候选帧，不等同于同步异常阈值。匹配采用前向游标：成功选中的帧会被消费，不会在后续分组中重复使用。

### 强制 335Lg 参考设备

多设备分组必须优先以 Gemini 335Lg 为参考设备：

- 设备名称含 `335Lg` 或 `335`；或
- PID 为 `2059`。

只有未检测到有效的 335Lg 帧时，才退回到“有效设备中帧数最少者”为参考设备。

完整多设备组的同步精度为：

```text
precisionUs = max(globalTimestampUs of group)
            - min(globalTimestampUs of group)
```

`--global-threshold=N` 默认 `5000 us`。若：

```text
precisionUs >= globalThresholdUs
```

则该组计为异常。该阈值与 `matchTolUs` 相互独立。

---

## 三、构建与运行

### 1. 部署到板端

当前板端项目路径为 `~/ob_time`。在 Windows PowerShell 中复制源文件：

```powershell
scp "D:\Data\robotPackage\ob_tools\src\main.cpp"           "mscape@192.168.137.2:/home/mscape/ob_time/main.cpp"
scp "D:\Data\robotPackage\ob_tools\src\data_collector.h"   "mscape@192.168.137.2:/home/mscape/ob_time/data_collector.h"
scp "D:\Data\robotPackage\ob_tools\src\data_collector.cpp" "mscape@192.168.137.2:/home/mscape/ob_time/data_collector.cpp"
scp "D:\Data\robotPackage\ob_tools\src\sync_analyzer.h"    "mscape@192.168.137.2:/home/mscape/ob_time/sync_analyzer.h"
scp "D:\Data\robotPackage\ob_tools\src\sync_analyzer.cpp"  "mscape@192.168.137.2:/home/mscape/ob_time/sync_analyzer.cpp"
scp "D:\Data\robotPackage\ob_tools\src\frame_stamp.h"      "mscape@192.168.137.2:/home/mscape/ob_time/frame_stamp.h"
```

如构建配置有变动，再复制 `CMakeLists.txt`。

### 2. 编译

```bash
ssh mscape@192.168.137.2
cd ~/ob_time
cmake --build build -j2
```

出现 `Built target timestamp_sync_check` 即表示编译完成。

### 3. 运行

从 `~/ob_time` 目录运行 30 秒受控基线：

```bash
sudo ./build/timestamp_sync_check \
  --duration=30 --fps=30 --width=1280 --height=800 \
  --global-threshold=5000 \
  --raw-csv=raw_30s.csv --csv=sync_result_30s.csv
```

若当前目录已经是 `~/ob_time/build`，可执行文件应写为：

```bash
sudo ./timestamp_sync_check ...
```

需要由程序自动管理外部 PWM 时，追加 `--trigger-hz=N`。写入 `/sys/kernel/debug/gpio_trigger/framerate` 通常需要 `sudo`。

---

## 四、命令行参数

| 参数 | 说明 | 默认值 |
|---|---|---:|
| `--duration=N` | 采集时长（秒） | 300 |
| `--fps=N` | 请求流帧率 | 30 |
| `--width=N` | 请求分辨率宽 | 1280 |
| `--height=N` | 请求分辨率高 | 800 |
| `--trigger-hz=N` | 自动控制外部 PWM；`0` 表示不接管 | 0 |
| `--global-threshold=N` | 多设备组 global 极差异常阈值（us） | 5000 |
| `--no-depth` | 不启用深度流 | 关闭 |
| `--no-color` | 不启用彩色流 | 关闭 |
| `--outdir=PATH` | 保存彩色 PNG 与图像时间戳 CSV | 不保存 |
| `--raw-csv=PATH` | 导出原始帧时间戳 CSV | 不导出 |
| `--csv=PATH` | 导出分析结果 CSV（必填） | — |
| `--help` | 显示帮助 | — |

> `main.cpp` 当前帮助文字仍显示旧的 `848x480` 默认值；实际采集默认值以 `DataCollector::Config` 的 `1280x800` 为准。

---

## 五、输出与检查项

### CSV 格式

`--raw-csv=PATH`：每一行是一帧原始记录。

```text
deviceIndex,streamType,hwTimestampUs,globalTimestampUs,sysTimestampUs,frameNumber
```

`--csv=PATH`：输出同设备跨流、跨设备同流和多设备分组的差值。

```text
comparison_type,device_i,device_j,stream,hw_diff_us,global_diff_us,sys_diff_us,timestamp_us
```

其中多设备行的 `global_diff_us` 是该组 `max(global)-min(global)`，而不是两设备间的有符号差。

### 板端基线检查

每次运行至少确认：

1. 设备 SN、型号、PID 与 global timestamp 支持状态正确；
2. 335Lg 的 FPS boost 读回为启用；
3. 读回同步配置为 hardware triggering、`triggerOut=1`、所有延迟为 0、`framesPerTrigger=1`；
4. 日志顺序为先启用 global timestamp、再同步设备时钟、再等待 1 秒；
5. 多设备报告的参考设备为 335Lg；
6. FrameSet 完整性、帧号/metadata 连续性、时间戳倒退与 cadence gap 诊断；
7. Depth 和 Color 的匹配组数、`max(global)-min(global)` 范围及 `>= 5000 us` 异常比例。

残余 global 时间戳偏差需要使用同一参数进行重复受控测试判断；不要在一次测试里同时改变同步配置、启动策略、参考设备、FPS boost、流 profile 或触发生命周期。
