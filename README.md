# 多相机时间戳同步校验工具（ob_tools）

基于 **Orbbec SDK** 的三相机硬件触发时间戳同步校验工具。程序采集 Depth 和 Color 帧，以全局时间戳（global timestamp）完成跨设备匹配，并输出同步精度统计与 CSV 数据。

## 1. 适用场景

当前实现面向以下硬件与约束：

- 2 × Gemini 305g + 1 × Gemini 335Lg；
- GMSL2 / FAKRA 外部硬件触发；
- 同步模式：`OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING`；
- 多设备匹配固定优先以 **Gemini 335Lg** 为参考设备；
- 使用 `Frame::globalTimeStampUs()` 进行匹配和同步精度计算。

> 本仓库不负责配置外部触发硬件、安装 Orbbec SDK 或部署开发环境；以下内容从“源码、SDK 与设备已经就绪”开始说明工具的构建与使用。

完整的同步配置、开流、诊断和 SDK API 说明请阅读：[同步配置、开流与全局时间戳匹配流程](docs/synchronization-flow.md)。

---

## 2. 仓库结构

```text
ob_tools/
├── src/                         # 独立 CMake 工程与主程序源码
│   ├── CMakeLists.txt            # timestamp_sync_check 构建入口
│   ├── main.cpp                  # CLI 参数解析与采集/分析编排
│   ├── data_collector.*          # 相机枚举、同步配置、开流和帧采集
│   ├── sync_analyzer.*           # global timestamp 匹配与统计
│   └── frame_stamp.h             # 单帧时间戳记录结构
├── docs/
│   └── synchronization-flow.md   # 同步全流程与 Orbbec API 参考
├── ob_official/                  # Orbbec 官方样例，用于行为对照
├── HANDOFF.md                    # 历史需求、约束和排查记录
└── README.md
```

仓库中可能还包含图像数据、CSV、图表和 Python 分析脚本。这些是实验数据或离线分析材料，**不属于构建 `timestamp_sync_check` 的必要输入**。

---

## 3. 运行前检查

### 3.1 软件依赖

构建 `src/` 中的独立 CMake 工程需要：

- CMake 3.10 或更高版本；
- 支持 C++17 的编译器；
- Orbbec SDK（需要提供 CMake 包 `OrbbecSDK`）；
- OpenCV（仅 `--outdir` 保存 PNG 时使用，但当前构建始终会链接 OpenCV）；
- Linux 下的 pthread。

`src/CMakeLists.txt` 默认从下列路径寻找 Orbbec SDK：

```text
$HOME/OrbbecSDK_v2.9.3_202607151523_2f6561c_linux_arm64/lib
```

SDK 安装在其他位置时，构建时显式传入其 `lib` 目录：

```bash
-DOrbbecSDK_DIR=/path/to/OrbbecSDK/lib
```

### 3.2 硬件与同步前提

启动程序前确认：

1. 至少有两台相机已被 SDK 识别；
2. 外部触发链路和相机连接正常；
3. 335Lg 可被识别为名称含 `335Lg` / `335` 或 PID `2059`；
4. 需要自动控制外部 PWM 时，当前用户有权限写入触发驱动节点；
5. 335Lg 的 FPS boost 可被设置为启用。

程序启动后会读取并输出设备信息、FPS boost 状态和硬件同步配置；应以这些读回日志为准，而非只检查命令行参数。

---

## 4. 构建

构建入口位于 `src/`。在仓库根目录执行：

```bash
cmake -S src -B build
cmake --build build -j2
```

若 Orbbec SDK 不在默认位置：

```bash
cmake -S src -B build \
  -DOrbbecSDK_DIR=/path/to/OrbbecSDK/lib
cmake --build build -j2
```

构建成功后，可执行文件为：

```text
build/timestamp_sync_check
```

可使用以下命令确认可执行文件和参数说明：

```bash
./build/timestamp_sync_check --help
```

> `--help` 中的宽高默认值目前仍显示旧的 `848x480` 文案；实际采集配置默认值以 `DataCollector::Config` 为准，即 `1280x800 @ 30 fps`。

---

## 5. 快速开始

### 5.1 最小采集与分析

以下命令使用外部已经运行的触发源，采集 30 秒，并输出原始帧 CSV 和分析结果 CSV：

```bash
sudo ./build/timestamp_sync_check \
  --duration=30 --fps=30 --width=1280 --height=800 \
  --global-threshold=5000 \
  --raw-csv=raw_30s.csv \
  --csv=sync_result_30s.csv
```

其中 `--csv` 为必填参数；`--raw-csv` 可选，但建议在排查同步问题时始终保留。

### 5.2 自动管理外部触发

如果平台支持通过程序写入外部 PWM 触发频率，增加 `--trigger-hz=N`：

```bash
sudo ./build/timestamp_sync_check \
  --duration=30 --fps=30 --width=1280 --height=800 \
  --trigger-hz=30 --global-threshold=5000 \
  --raw-csv=raw_30s.csv \
  --csv=sync_result_30s.csv
```

自动触发模式的生命周期为：

```text
写 0 停止自动触发
→ 启动全部 Pipeline
→ 等待约 300 ms
→ 写入 N 开始触发
→ 采集结束后先停止 Pipeline
→ 最后写 0 关闭触发
```

写入触发驱动节点通常需要 `sudo`。若未指定 `--trigger-hz`，默认值为 `0`，程序不会接管触发频率。

### 5.3 保存彩色图像

指定 `--outdir` 后，程序额外保存采集到的彩色帧为 PNG，并在该目录生成图像时间戳 CSV：

```bash
sudo ./build/timestamp_sync_check \
  --duration=10 --fps=30 --width=1280 --height=800 \
  --outdir=output \
  --raw-csv=raw.csv --csv=sync.csv
```

PNG 写盘在后台线程执行，避免慢速磁盘 I/O 直接阻塞 SDK 回调。图像索引使用的时间戳不能替代 global timestamp 同步分析结果。

---

## 6. 命令行参数

| 参数 | 说明 | 默认值 |
|---|---|---:|
| `--duration=N` | 采集时长，单位秒 | 300 |
| `--fps=N` | 请求的 Depth / Color 流帧率 | 30 |
| `--width=N` | 请求分辨率宽 | 1280 |
| `--height=N` | 请求分辨率高 | 800 |
| `--trigger-hz=N` | 自动控制外部 PWM；`0` 表示不接管 | 0 |
| `--global-threshold=N` | 多设备 global 极差异常阈值，单位 us | 5000 |
| `--no-depth` | 不启用 Depth 流 | 关闭 |
| `--no-color` | 不启用 Color 流 | 关闭 |
| `--outdir=PATH` | 保存彩色 PNG 和图像时间戳 CSV | 不保存 |
| `--raw-csv=PATH` | 导出每帧原始时间戳 | 不导出 |
| `--csv=PATH` | 导出分析结果 CSV（必填） | — |
| `--help` | 显示帮助 | — |

---

## 7. 同步规则与结果含义

### 7.1 时间戳域

每一帧会记录以下字段：

| 字段 | SDK API | 用途 |
|---|---|---|
| `hwTimestampUs` | `frame->timeStampUs()` | 相机本地硬件时间戳，仅用于对照与诊断 |
| `globalTimestampUs` | `frame->globalTimeStampUs()` | **跨设备匹配与同步精度的主时间戳** |
| `sysTimestampUs` | `frame->systemTimeStampUs()` | 主机接收与回调诊断 |
| `frameNumber` | `frame->getIndex()` | 帧连续性诊断 |

程序按 `globalTimestampUs` 排序和匹配。硬件时间戳、系统时间戳不会作为多设备同步精度的判定依据。

### 7.2 335Lg 强制参考设备

多设备分组时，程序必须优先使用 Gemini 335Lg 作为参考设备：

- 设备名包含 `335Lg` 或 `335`；或
- PID 为 `2059`。

只有没有检测到有效 335Lg 帧时，才回退到“有效设备中帧数最少者”作为参考设备。

### 7.3 配对容差与异常阈值

帧配对使用约半帧周期作为候选容差：

```text
matchTolUs = round(1,000,000 / fps / 2)
```

在 30 fps 时约为 `16667 us`。成功配对后，帧会被消费，不会重复用于后续分组。

完整多设备组的同步精度定义为：

```text
precisionUs = max(globalTimestampUs of group)
            - min(globalTimestampUs of group)
```

当：

```text
precisionUs >= globalThresholdUs
```

该组记为异常。默认 `globalThresholdUs=5000 us`。注意：配对容差 `matchTolUs` 用于寻找候选帧，异常阈值用于评价已组成分组的同步精度；两者不是同一个参数。

---

## 8. 输出文件

### 8.1 原始帧 CSV：`--raw-csv`

```text
deviceIndex,streamType,hwTimestampUs,globalTimestampUs,sysTimestampUs,frameNumber
```

- 一行对应一帧原始记录，不代表已经完成匹配；
- 离线分析跨设备同步时应使用 `globalTimestampUs`；
- `hwTimestampUs`、`sysTimestampUs` 和 `frameNumber` 用于定位跳帧、回调积压或时间轴问题。

### 8.2 分析结果 CSV：`--csv`

```text
comparison_type,device_i,device_j,stream,hw_diff_us,global_diff_us,sys_diff_us,timestamp_us
```

`comparison_type` 包含：

- `cross_stream`：同一设备的 Depth 与 Color；
- `cross_device`：不同设备的同类流；
- `multi_device`：所有参与设备的完整同步组。

对 `multi_device` 行，`global_diff_us` 是组内 `max(global)-min(global)` 的非负极差；它不是两个设备间带正负号的差值。

---

## 9. 每次测试的检查清单

建议在每次基线测试后确认控制台日志：

1. 设备 SN、型号、PID 和 global timestamp 支持状态正确；
2. 335Lg 的 `OB_PROP_FPS_BOOST_BOOL` 读回为启用；
3. 读回同步配置为 hardware triggering、`triggerOut=1`、所有延迟为 `0`、`framesPerTrigger=1`；
4. global timestamp 初始化顺序为：全部设备启用 global timestamp → device clock sync → 等待 1 秒；
5. 多设备分析报告中的参考设备为 335Lg；
6. Depth / Color 的帧数、FrameSet 完整性、帧序号与 metadata 连续性、时间戳倒退和 cadence gap 诊断；
7. Depth / Color 的匹配组数、`max(global)-min(global)` 范围，以及 `>= 5000 us` 的异常比例。

当前 Pipeline 按设备索引顺序启动。这是用于检验启动顺序是否影响残余 global 相位关系的单变量实验，**不是已验证的同步修复**。比较结果时不要同时改动同步配置、启动策略、参考设备、FPS boost、流 profile 或触发生命周期。

---

## 10. 常见问题

| 现象 | 检查方式 |
|---|---|
| 构建时找不到 `OrbbecSDK` | 使用 `-DOrbbecSDK_DIR=/path/to/OrbbecSDK/lib` 指向 SDK 的 `lib` 目录。 |
| 构建时找不到 OpenCV | 安装开发包，并确认 `find_package(OpenCV REQUIRED)` 可用。 |
| 程序提示至少需要两台设备 | 检查所有相机的物理连接、供电和 SDK 枚举日志。 |
| 335Lg 在 30 Hz 下帧数约为其他相机的一半 | 检查 `OB_PROP_FPS_BOOST_BOOL` 是否成功设置并读回为 `true`。 |
| 使用 `--trigger-hz` 但无法写触发节点 | 检查触发驱动节点、debugfs 状态和执行权限；通常需要 `sudo`。 |
| 多设备结果异常比例较高 | 先保存 `--raw-csv`，检查同步配置读回、global 时钟初始化顺序、335Lg 参考选择、FrameSet 完整性与帧号/cadence 诊断。 |
| 在 `build/` 目录运行时找不到程序 | 从仓库根目录使用 `./build/timestamp_sync_check`；从 `build/` 内使用 `./timestamp_sync_check`。 |

如需了解硬件同步配置字段、全局时钟初始化、开流 gate、停流顺序及 Orbbec API 的详细语义，请参阅：[docs/synchronization-flow.md](docs/synchronization-flow.md)。
