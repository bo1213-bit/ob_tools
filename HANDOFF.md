# 时间戳同步分析工具 — 交接文档 (Handoff)

> 目标读者：下一个接手此任务的 AI。请先读完本文再动手。

## 1. 任务背景

在 Orbbec 三相机硬件触发 (HW trigger) 同步场景下，写一个时间戳同步精度分析工具，目标是达到官方 ~3ms 的同步效果。

- 运行平台：tegra/Orin 板，IP `192.168.137.2`，用户 `mscape`，板端源码目录 `~/ob_time/`
- 本地 Windows 工作副本：`D:\Data\robotPackage\ob_tools`
- 相机：dev0 (Gemini 305g, SN CV3T561000B5, PID 2114)、dev1 (Gemini 305g, SN CV3T561000H0, PID 2114)、dev2 (Gemini 335Lg, SN CPBG1630011D, PID 2059)。全部 GMSL2、硬件触发模式。
- 技术栈：C++17，Orbbec SDK v2.9.3

## 2. 三个时间戳字段（务必区分）

| 字段 | C++ API | 含义 |
|------|---------|------|
| `hwTimestampUs` | `frame->timeStampUs()` | 设备本地硬件时钟时间戳 |
| `globalTimestampUs` | `frame->globalTimeStampUs()` | 换算到主机时钟域的时间戳 |
| `sysTimestampUs` | `frame->systemTimeStampUs()` | 主机收到帧的时间 |

另外 `frameNumber` = `frame->getIndex()`。

## 3. 用户的关键指令（本任务的核心决策）

用户原话（翻译）：

> "等一下，是用 globalTimestampUs（全局时间戳）来做最近邻配对。之前的代码是不是用 globaltime 来做匹配之后、使用 hwtime 来输出？现在改成用 globaltime 来做匹配，然后同样使用 globaltime 来作为时间戳精度。"

**结论（必须遵守）：**
1. **最近邻配对** 用 `globalTimestampUs`（不是 `hwTimestampUs`）。
2. **同步精度度量/输出** 也用 `globalTimestampUs`（不是 `hwTimestampUs`）。
3. `hwTimestampUs` 降级为"仅参考"。

这与官方 `analyze_sync.py` 一致：官方 `DEFAULT_TIMESTAMP_SOURCE = "auto"` 会解析为 `"global"`（仅当 global 全 0 时才退回 `"device"`）。

## 4. 为什么之前一直达不到官方效果（根因）

之前 `max Global Timestamp Diff` 到 ~16.6ms（半帧临界），根因是 **global 时间戳在漂移**，而漂移来自采集端调用顺序错误：

- 旧顺序：先 `enableDeviceClockSync` 再 `enableGlobalTimestamp`，且没有等待稳定时间。
- 官方正确顺序（`MultiDeviceSync.cpp` 的 `testMultiDeviceSync()`）：
  1. 每个设备 `device->enableGlobalTimestamp(true)`
  2. `context.enableDeviceClockSync(0)`
  3. `std::this_thread::sleep_for(std::chrono::seconds(1))`（等时钟同步稳定）
  4. 然后再开始取流

不按这个顺序，global 在 30s 内会漂移 ~40ms。

## 5. 两个阈值（不要混淆）

- `matchTolUs = 1e6/fps/2`（半帧间隔，@30fps ≈ 16667us）—— 贪婪匹配的容差，与官方 `half_gap_us` 一致。
- `globalThresholdUs`（本工具 `--global-threshold` 参数，默认 5000us）—— 匹配后判定"异常"的阈值，作用于 global 时间戳极差。**注意：官方默认是 2000us**，本工具用了 5000us，改完后用户可能要下调到 2000。

## 6. 已完成的工作（本次会话已改）

### 6.1 `src/data_collector.cpp` — `resetTimestampAndSyncClock()`（已改，完成）

已重排为官方顺序：
```cpp
// 1) 先 enableGlobalTimestamp(true) 每个设备
// 2) 再 context_->enableDeviceClockSync(0)
// 3) sleep 1 秒
```
这样 global 时间戳才稳定，是后面一切正确性的前提。

### 6.2 `src/sync_analyzer.cpp`（已改，完成）

- `matchAndDiff()`：已改回按 `globalTimestampUs` 匹配（注释也更新）。
- `multiDeviceMatch()`：已改回按 `globalTs`/`gArr` 匹配；返回 `result.global`（全局极差，用于精度）和 `result.hw`（hw 极差，仅参考）。
- `run()` 里的 `calcMdStats(...)`：两处（Depth、Color）都已从 `.hw` 改为 `.global`，"异常"计数现在作用于 global 极差。
- `printOneStats()`：标签已对调 —— 主指标改为 "Global Timestamp Diff (sync precision, official time base)"，副指标改为 "Device (hw) Timestamp Diff (ref only)"。
- `printReport()` 的 `printMd`：标签已改为 `max(global)-min(global)`。
- 各处注释里 "按设备端 hw 匹配" 已改为 "按全局 global 匹配"。

### 6.3 `src/sync_analyzer.h`（已改，完成）

- `PairStats` 字段注释已对调：`global*` = "同步精度的正确度量"，`hw*` = "仅参考"。
- `MultiDeviceStats` 字段：`hwMinUs/hwMaxUs/hwMeanUs/hwStddevUs` 已重命名为 `globalMinUs/globalMaxUs/globalMeanUs/globalStddevUs`（注释：每组全局时间戳极差的最小/最大值，同步精度正确度量）。
- `matchAndDiff` / `multiDeviceMatch` / `MultiDeviceDiffs` 的注释都已更新为 global-primary。

## 7. 遗留 / 待办

1. **编译校验**：改动未在板端编译过，需确认 `MultiDeviceStats` 字段改名（hw→global）后，`sync_analyzer.cpp` 与 `.h` 完全一致、无残留 `s.hwMinUs` 引用。本地已 grep 过：`.cpp` 中无残留 `s.hwMinUs/hwMaxUs`（只剩 `PairStats` 的 hw 字段，属正常）；`calcMdStats` 已用 `s.globalMinUs` 等。但**建议下个 AI 在板端实际编译一遍确认**。

2. **`src/visualize_timestamps.py` 仍用 hw**（第 57、84 行 `max(hw)-min(hw)`）。这是给 `TimestampRecorder` 输出画直方图的脚本，数据源与主工具不同。是否需要同样改成 global，需用户确认——若该 recorder 的 CSV 里 global 稳定，应一并改。

3. **阈值默认值**：`--global-threshold` 默认 5000us，官方默认 2000us。若要看官方一致的"异常"比例，建议下调到 2000。

4. **给用户的重编译 + 运行指引**（见下节）。

## 8. 板端重编译 + 运行步骤（给用户）

```bash
# 1) 把本地改好的三个文件 scp 到板端 ~/ob_time/ 对应位置
scp src/sync_analyzer.cpp src/sync_analyzer.h src/data_collector.cpp mscape@192.168.137.2:~/ob_time/src/

# 2) 板端重新编译
ssh mscape@192.168.137.2
cd ~/ob_time
touch src/sync_analyzer.cpp src/data_collector.cpp
make -j2

# 3) 重新运行采集 + 分析
#   （具体命令以项目现有 Makefile/运行脚本为准）
```

## 9. 期望结果

改完后（采集端按官方顺序初始化 + 匹配与精度都用 global），预期：
- 匹配对数接近满帧；
- `max(global)-min(global)` 的 Min/Max 应回落到 ~3ms 量级（而非此前的 ~16.6ms）；
- "Abnormal" 比例应明显下降。

如果 global 仍然漂移、极差仍然大，则回头检查 `resetTimestampAndSyncClock()` 是否真的按官方顺序执行、以及 `enableDeviceClockSync(0)` 是否调用成功（返回值）。
