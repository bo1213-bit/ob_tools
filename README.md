# 多相机时间戳同步校验工具 (ob_tools)

基于 OrbbecSDK 的多相机硬件触发时间戳同步校验程序。

- **硬件**：3 台 Orbbec 相机（2× Gemini 305g + 1× Gemini 335Lg），GMSL2 / FAKRA 接口
- **触发方式**：外部 PWM 硬件触发（`OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING`）
- **核心目标**：验证三台相机在同一触发沿下，时间戳是否对齐

---

## 一、业务逻辑

### 1.1 整体流程

程序由两个模块编排而成，入口在 `main.cpp`：

```
DataCollector（采集）  →  SyncAnalyzer（分析）  →  输出报告 + CSV
```

### 1.2 采集流程（`data_collector.cpp`）

`DataCollector::run()` 按以下顺序执行：

1. **枚举设备** `enumerateDevices()`
   - 列出所有相机（SN / 型号 / PID / VID / 固件）
   - 检查是否支持全局时间戳（global timestamp）

2. **配置硬件同步** `configureSyncMode()`
   - 所有相机设为 `HARDWARE_TRIGGERING`（外部硬件触发）
   - `triggerOutEnable = false`（不产生级联触发，由外部 PWM 统一驱动）

3. **时钟对齐** `resetTimestampAndSyncClock()`
   - 逐台 `timerSyncWithHost()`（设备与主机时钟一次性对齐）
   - `enableGlobalTimestamp(true)`：把设备本地时间戳换算到主机时钟域，用于跨设备对齐

4. **采集帧** `collectFrames()`
   - 每个相机开一条 `ob::Pipeline`（Depth + Color 双流）
   - **3 线程 + 主线程发令枪**：三台相机同时 `start()`
   - 回调里记录每帧三类时间戳：
     - `hwTimestampUs` — 设备端硬件时间戳
     - `globalTimestampUs` — 换算到主机时钟域的全局时间戳
     - `sysTimestampUs` — 主机端系统时间戳
   - 采集固定时长后停止

### 1.3 触发自动控制（关键改进）

`--trigger-hz=N`（N > 0）时，程序**自动接管 PWM 触发开关**，解决"相机先后加入触发流导致帧数不均"的问题：

```
① 程序启动 → 写 0 关触发（清空旧触发流）
② 枚举 / 配置 / 三台 start() 就绪（此时无触发，不出帧）
③ 等待 300ms settle（确保相机进入可触发状态）
④ 写 N 开触发 ← 发令枪，三台从同一触发沿开始出帧
⑤ 采集 N 秒
⑥ 停 pipeline → 写 0 关触发收尾
```

> 为什么必须"先 arm、再触发"：硬件触发模式下，相机只在收到触发脉冲时才出帧。若 PWM 先于程序在跑，三台相机 `start()` 内部耗时不同、先后加入触发流，导致帧数错开、帧号无法一一对应。只有"三台先就绪、再统一放触发"，才能保证从同一触发沿开始、帧数严格相等。

### 1.4 分析流程（`sync_analyzer.cpp`）

`SyncAnalyzer::run()` 做四类对比：

| 对比类型 | 说明 |
|---|---|
| 1. 同设备跨流 | 单台相机 Depth vs Color |
| 2. 跨设备 Depth | 相机两两 Depth vs Depth（用 globalTimestampUs 匹配） |
| 3. 跨设备 Color | 相机两两 Color vs Color（用 globalTimestampUs 匹配） |
| 4. 多设备同步 | 所有相机同一流匹配后 `max(hw) - min(hw)` |

每种对比输出统计量：`min / max / mean / stddev`（硬件时间戳差 + 系统时间戳差）。

---

## 二、执行指令

### 2.1 部署（Windows → 远端 Linux）

在 Windows 的 PowerShell 里 `scp` 四个源文件到远端：

```powershell
scp "D:\Data\robotPackage\ob_tools\src\main.cpp"             user@192.168.137.6:ros2_ws/src/main.cpp
scp "D:\Data\robotPackage\ob_tools\src\data_collector.h"     user@192.168.137.6:ros2_ws/src/data_collector.h
scp "D:\Data\robotPackage\ob_tools\src\data_collector.cpp"   user@192.168.137.6:ros2_ws/src/data_collector.cpp
scp "D:\Data\robotPackage\ob_tools\src\sync_analyzer.h"      user@192.168.137.6:ros2_ws/src/sync_analyzer.h
scp "D:\Data\robotPackage\ob_tools\src\sync_analyzer.cpp"    user@192.168.137.6:ros2_ws/src/sync_analyzer.cpp
scp "D:\Data\robotPackage\ob_tools\src\frame_stamp.h"        user@192.168.137.6:ros2_ws/src/frame_stamp.h
scp "D:\Data\robotPackage\ob_tools\src\CMakeLists.txt"       user@192.168.137.6:ros2_ws/src/CMakeLists.txt
```

### 2.2 编译

```bash
ssh user@192.168.137.6
cd ~/ros2_ws/src/build
cmake .. && make
```

看到 `Built target timestamp_sync_check` 且无 `error:` 即成功。

### 2.3 运行（硬件触发自动控制，推荐）

必须用 `sudo`（写 `/sys/kernel/debug/gpio_trigger/framerate` 需要 root）：

```bash
sudo ./timestamp_sync_check \
  --csv=./sync.csv \
  --raw-csv=./raw.csv \
  --duration=5 \
  --width=1280 \
  --height=800 \
  --trigger-hz=10
```

### 2.4 运行（不接管触发，纯采集）

`--trigger-hz=0`（默认）时不碰 PWM，需手动控制触发：

```bash
./timestamp_sync_check --csv=./sync.csv --raw-csv=./raw.csv \
  --duration=5 --width=1280 --height=800
```

---

## 三、命令行参数

| 参数 | 说明 | 默认 |
|---|---|---|
| `--duration=N` | 采集时长（秒） | 300 |
| `--fps=N` | 流帧率 | 30 |
| `--width=N` | 分辨率宽 | 848 |
| `--height=N` | 分辨率高 | 480 |
| `--trigger-hz=N` | 自动控制外部 PWM 触发频率（0=关闭控制） | 0 |
| `--hw-threshold=N` | 硬件时间戳配对阈值（us） | 500 |
| `--no-depth` | 关闭深度流 | 关 |
| `--no-color` | 关闭彩色流 | 关 |
| `--outdir=PATH` | 保存彩色帧为 PNG + timestamps.csv | 不保存 |
| `--raw-csv=PATH` | 导出原始帧时间戳 CSV | 不导出 |
| `--csv=PATH` | 分析结果 CSV 路径（**必填**） | — |
| `--help` | 帮助 | — |

---

## 四、输出说明

- **`raw.csv`**（`--raw-csv`）：每帧原始时间戳
  `deviceIndex,streamType,hwTimestampUs,globalTimestampUs,sysTimestampUs`
- **`sync.csv`**（`--csv`）：四类对比的原始 diff
  `comparison_type,device_i,device_j,stream,hw_diff_us,global_diff_us,sys_diff_us,timestamp_us`

### 验证结果怎么看

1. **帧数**：`--trigger-hz=10`、`--duration=5` 时，三台 Depth 应 ≈50 帧且彼此相等。
2. **帧间隔**（验证触发频率）：

```bash
awk -F, '$1==0 && $2=="DEPTH" {print $4}' raw.csv
```

相邻值相减，应约等于 `1000000 / trigger-hz` us（10Hz → 100000us）。

---

## 五、常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `[WARN] cannot open trigger node` | debugfs 未挂载或没加 `sudo` |
| 写 0 之后相机仍出帧 | 0 不是"停触发"语义，需确认节点含义 |
| 帧数不均 | 触发先于程序在跑；改用 `--trigger-hz` 自动控制 |
| 帧率对不上（如 10Hz 出了 15Hz） | 检查 `cat /sys/kernel/debug/gpio_trigger/framerate`，确认单位是 Hz |
