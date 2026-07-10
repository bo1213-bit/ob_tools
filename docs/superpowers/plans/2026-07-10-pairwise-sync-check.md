# Pairwise System Timestamp Sync Check — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `timestamp_sync_check.cpp` 从 "2 相机 master/slave 硬件时间戳对比" 重构为 "N 相机 pairwise 软件端到端时间戳精度验证"。

**Architecture:** 用 `std::vector` 统一管理 N 台设备的 Device/Pipeline/Queue/Mutex，时间戳来源从 `getTimeStampUs()`/`getSystemTimeStampUs()` 改为 `OB_FRAME_METADATA_TYPE_TIMESTAMP`/`std::chrono::system_clock::now()`，配对改为全矩阵 pairwise，统计输出 CSV + 终端报告。

**Tech Stack:** C++11, Orbbec SDK v2, CMake

## Global Constraints

- 不修改 CMakeLists.txt
- 不引入新的第三方库（argc/argv 手动解析）
- 保持 C++11 标准
- 不新增 .cpp/.h 文件（除 README.md）
- 异常传播模式不变：main() 统一 catch
- 方法调用顺序固定：enumerateDevices → configureSyncMode → resetTimestampAndSyncClock → collectFrames → matchAndComputeDiffs → printStatistics

---

## File Structure

| 文件 | 职责 |
|------|------|
| `timestamp_sync_check.cpp` | 唯一修改文件。包含：FrameStamp 结构体、g_running/signalHandler、TimestampSyncChecker 类、main() + argc/argv 解析 |
| `README_timestamp_sync_check.md` | 使用说明文档（新增） |

---

### Task 1: 重构数据结构 — master/slave → N 设备 vector + 时间戳来源改为 metadata/chrono

**Files:**
- Modify: `timestamp_sync_check.cpp`（整个文件重写）

**Interfaces:**
- Consumes: 无
- Produces: 新的成员变量和 FrameStamp 采集逻辑
  - `std::vector<std::shared_ptr<ob::Device>> devices_`（替代 `masterDev_`/`slaveDev_`）
  - `std::vector<std::shared_ptr<ob::Pipeline>> pipelines_`（替代 `masterPipe_`/`slavePipe_`）
  - `std::vector<std::queue<FrameStamp>> queues_`（替代 `masterQueue_`/`slaveQueue_`）
  - `std::vector<std::mutex> mutexes_`（替代 `masterMutex_`/`slaveMutex_`）
  - `std::vector<std::vector<FrameStamp>> allFrames_`（替代 `masterFrames_`/`slaveFrames_`）
  - `std::map<std::pair<int,int>, std::vector<int64_t>> pairDiffs_`（替代 `diffs_`）
  - 时间戳来源变更：hw = `frame->getMetadataValue(OB_FRAME_METADATA_TYPE_TIMESTAMP)`，sys = `std::chrono::system_clock::now()`（回调第一行）

- [ ] **Step 1: 重写文件头部注释、include、FrameStamp**

将 `timestamp_sync_check.cpp` 的第 1-40 行替换为：

```cpp
// timestamp_sync_check.cpp
//
// Pairwise system-timestamp sync check for hardware-synced Orbbec cameras.
// Verifies software end-to-end sync accuracy < 1ms across all camera pairs.
//
// 硬件时间戳 (OB_FRAME_METADATA_TYPE_TIMESTAMP) 用于帧配对。
// 软件时间戳 (std::chrono::system_clock::now() 在 SDK 回调入口记录) 用于对比。
//
// Step 1: Enumerate devices and print device info
// Step 2: Configure first device PRIMARY, rest SECONDARY
// Step 3: Timestamp reset + device clock sync
// Step 4: Start pipelines, collect frames in callbacks
// Step 5: Pairwise frame matching by hw timestamp, compute sys timestamp diffs
// Step 6: Statistics output (per-pair + global + histogram + CSV)

#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// Each frame's timestamp info we store in the queue
// ============================================================================
struct FrameStamp {
    int64_t hwTimestampUs;   // OB_FRAME_METADATA_TYPE_TIMESTAMP (for pairing)
    int64_t sysTimestampUs;  // std::chrono::system_clock at callback entry (for diff)
    int64_t frameNumber;     // OB_FRAME_METADATA_TYPE_FRAME_NUMBER
};

// ============================================================================
// Global state for the callback threads
// ============================================================================
static std::atomic<bool> g_running{true};

static void signalHandler(int /*signum*/) {
    g_running = false;
    std::cout << "\n[INFO] Received signal, shutting down..." << std::endl;
}
```

- [ ] **Step 2: 重写类定义和私有成员**

替换第 55-366 行（整个类定义和私有成员），先替换类声明和成员变量部分。在第 51 行 `}` (signalHandler 结束) 之后写入：

```cpp
// ============================================================================
// TimestampSyncChecker — orchestrates the 6-step pairwise sync check
// ============================================================================
class TimestampSyncChecker {
public:
    // Runtime parameters (set from command line)
    int  durationSec    = 30;
    int  hwThresholdUs  = 500;
    int  syncThresholdUs = 1000;
    int  fps            = 30;
    int  width          = 640;
    int  height         = 400;
    std::string csvPath;
    int  histogramBins  = 10;
    bool histogramAll   = false;

    // ---- Step 1: Enumerate devices ----
    void enumerateDevices() {
        context_ = std::make_shared<ob::Context>();
        auto devList = context_->queryDeviceList();
        int devCount = devList->deviceCount();
        std::cout << "Found " << devCount << " device(s)" << std::endl;

        for (int i = 0; i < devCount; i++) {
            auto dev  = devList->getDevice(i);
            auto info = dev->getDeviceInfo();
            std::cout << "Device " << i << ": "
                      << "SN=" << info->serialNumber()
                      << "  Name=" << info->getName()
                      << "  PID=" << info->getPid()
                      << "  VID=" << info->getVid()
                      << std::endl;
            auto syncBitmap = dev->getSupportedMultiDeviceSyncModeBitmap();
            std::cout << "  Supported sync modes: 0x" << std::hex << syncBitmap << std::dec << std::endl;
            devices_.push_back(dev);
        }

        if (devCount < 2) {
            throw std::runtime_error("Need at least 2 devices for sync check!");
        }
        std::cout << "\nDevices: " << devices_.size()
                  << "  Pairs to check: " << (devices_.size() * (devices_.size() - 1) / 2)
                  << std::endl;
    }

    // ---- Step 2: Configure sync mode ----
    void configureSyncMode() {
        for (size_t i = 0; i < devices_.size(); i++) {
            OBMultiDeviceSyncConfig cfg = devices_[i]->getMultiDeviceSyncConfig();
            if (i == 0) {
                // Primary
                cfg.syncMode             = OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
                cfg.triggerOutEnable     = true;
                cfg.triggerOutDelayUs    = 0;
            } else {
                // Secondary
                cfg.syncMode             = OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
                cfg.triggerOutEnable     = false;
                cfg.triggerOutDelayUs    = 0;
            }
            cfg.depthDelayUs         = 0;
            cfg.colorDelayUs         = 0;
            cfg.trigger2ImageDelayUs = 0;
            cfg.framesPerTrigger     = 1;
            devices_[i]->setMultiDeviceSyncConfig(cfg);

            auto sn = devices_[i]->getDeviceInfo()->serialNumber();
            std::cout << "Device " << i << " (SN=" << sn << "): "
                      << (i == 0 ? "PRIMARY" : "SECONDARY")
                      << "  triggerOut=" << (i == 0 ? "true" : "false") << std::endl;
        }

        // Verify
        std::cout << "\nVerify config:" << std::endl;
        for (size_t i = 0; i < devices_.size(); i++) {
            auto check = devices_[i]->getMultiDeviceSyncConfig();
            std::cout << "  Device " << i << " syncMode=" << check.syncMode
                      << "  triggerOut=" << check.triggerOutEnable << std::endl;
        }
    }

    // ---- Step 3: Timestamp reset + device clock sync ----
    void resetTimestampAndSyncClock() {
        devices_[0]->setBoolProperty(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, true);
        devices_[0]->setIntProperty(OB_PROP_TIMER_RESET_DELAY_US_INT, 20);
        devices_[0]->setBoolProperty(OB_PROP_TIMER_RESET_SIGNAL_BOOL, true);
        std::cout << "\nTimestamp reset sent (primary -> all secondaries, delay=20us)" << std::endl;

        context_->enableDeviceClockSync(100);
        std::cout << "Device clock sync enabled (every 100ms)" << std::endl;
    }

private:
    std::shared_ptr<ob::Context>               context_;
    std::vector<std::shared_ptr<ob::Device>>   devices_;
    std::vector<std::shared_ptr<ob::Pipeline>> pipelines_;
    std::vector<std::queue<FrameStamp>>        queues_;
    std::vector<std::mutex>                    mutexes_;
    std::vector<std::vector<FrameStamp>>       allFrames_;
    std::map<std::pair<int,int>, std::vector<int64_t>> pairDiffs_;
    int missedCount_ = 0;
};
```

- [ ] **Step 3: 验证编译环境检查**

确认文件前半部分（include、FrameStamp、signal handler、类声明、Step 1-3 方法、private 成员）无语法错误。如果设备上有编译器，运行：
```bash
cd ob_tools && mkdir -p build && cd build
cmake .. -DOrbbecSDK_DIR=../OrbbecSDK_v2.8.7_202606161335_ab8672c_linux_arm64/lib
make -j$(nproc) 2>&1 | head -30
```
预期：`Step 4-6` 方法缺失导致的链接错误是正常的，只要语法编译通过即可。

- [ ] **Step 4: Commit**

```bash
git add timestamp_sync_check.cpp
git commit -m "refactor: replace master/slave with N-device vector architecture

- devices_/pipelines_/queues_/mutexes_ become std::vector
- hw timestamp from OB_FRAME_METADATA_TYPE_TIMESTAMP
- sys timestamp from std::chrono::system_clock::now()
- Step 1-3 updated for N devices
- Pairwise data structures in place

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 2: 实现 Step 4 — N 设备采集帧（回调 sys 时间戳 + hw metadata）

**Files:**
- Modify: `timestamp_sync_check.cpp`

**Interfaces:**
- Consumes: Task 1 的 `devices_`, `queues_`, `mutexes_`, `allFrames_` 成员
- Produces: `void collectFrames(int durationSec)` — 启动 N 条 pipeline，回调中记录 sys(chrono) + hw(metadata)，采集后搬队列到 allFrames_

- [ ] **Step 1: 添加 collectFrames 方法到类中**

在 `resetTimestampAndSyncClock()` 方法之后、`private:` 之前，插入 `collectFrames`：

```cpp
    // ---- Step 4: Start pipelines, collect frames, stop, drain queues ----
    void collectFrames() {
        int deviceCount = static_cast<int>(devices_.size());

        // Resize per-device storage
        pipelines_.resize(deviceCount);
        queues_.resize(deviceCount);
        mutexes_.resize(deviceCount);
        allFrames_.resize(deviceCount);

        // Start all pipelines
        for (int i = 0; i < deviceCount; i++) {
            pipelines_[i] = std::make_shared<ob::Pipeline>(devices_[i]);
            auto cfg = std::make_shared<ob::Config>();
            cfg->enableVideoStream(OB_STREAM_DEPTH, width, height, fps, OB_FORMAT_Y16);

            int camIndex = i;  // capture by value for lambda
            pipelines_[i]->start(cfg, [this, camIndex](std::shared_ptr<ob::FrameSet> frameSet) {
                // sys timestamp: record NOW before any other work
                auto nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

                auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                if (!depthFrame) return;

                FrameStamp fs;
                fs.sysTimestampUs = nowUs;
                fs.hwTimestampUs  = depthFrame->getMetadataValue(OB_FRAME_METADATA_TYPE_TIMESTAMP);
                fs.frameNumber    = depthFrame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER);

                std::lock_guard<std::mutex> lock(mutexes_[camIndex]);
                queues_[camIndex].push(fs);
            });

            auto sn = devices_[i]->getDeviceInfo()->serialNumber();
            std::cout << "Device " << i << " (SN=" << sn << ") pipeline started (depth "
                      << width << "x" << height << " @ " << fps << "fps)" << std::endl;
        }

        // Collect frames
        std::cout << "\nCollecting frames for " << durationSec
                  << " seconds... (Ctrl+C to stop early)" << std::endl;
        auto startTime = std::chrono::steady_clock::now();
        while (g_running) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - startTime).count();
            if (elapsed >= durationSec) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop pipelines (reverse order)
        for (int i = deviceCount - 1; i >= 0; i--) {
            pipelines_[i]->stop();
        }
        std::cout << "Pipelines stopped" << std::endl;

        // Print and drain
        for (int i = 0; i < deviceCount; i++) {
            std::lock_guard<std::mutex> lock(mutexes_[i]);
            std::cout << "Device " << i << " frames collected: " << queues_[i].size() << std::endl;
            while (!queues_[i].empty()) {
                allFrames_[i].push_back(queues_[i].front());
                queues_[i].pop();
            }
        }

        int totalFrames = 0;
        for (int i = 0; i < deviceCount; i++) totalFrames += static_cast<int>(allFrames_[i].size());
        std::cout << "Total frames transferred: " << totalFrames << std::endl;
    }
```

- [ ] **Step 2: Commit**

```bash
git add timestamp_sync_check.cpp
git commit -m "feat: implement N-device frame collection with metadata timestamps

- sys timestamp via std::chrono::system_clock::now() at callback entry
- hw timestamp via OB_FRAME_METADATA_TYPE_TIMESTAMP
- Per-camera queues with independent mutexes
- Frame number via OB_FRAME_METADATA_TYPE_FRAME_NUMBER

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 3: 实现 Step 5 — 全矩阵 Pairwise 帧配对

**Files:**
- Modify: `timestamp_sync_check.cpp`

**Interfaces:**
- Consumes: Task 2 的 `allFrames_`（N 个 vector<FrameStamp>）
- Produces: `void matchAndComputeDiffs()` — 填充 `pairDiffs_` 和 `missedCount_`

- [ ] **Step 1: 在 collectFrames 之后添加 matchAndComputeDiffs 方法**

```cpp
    // ---- Step 5: Pairwise frame matching by hw timestamp, compute sys diffs ----
    void matchAndComputeDiffs() {
        int deviceCount = static_cast<int>(allFrames_.size());
        missedCount_ = 0;

        for (int i = 0; i < deviceCount; i++) {
            for (int j = i + 1; j < deviceCount; j++) {
                auto& framesI = allFrames_[i];
                auto& framesJ = allFrames_[j];
                auto key = std::make_pair(i, j);
                pairDiffs_[key] = std::vector<int64_t>();

                for (size_t mi = 0; mi < framesI.size(); mi++) {
                    int64_t bestDist = INT64_MAX;
                    size_t  bestIdx  = 0;

                    for (size_t mj = 0; mj < framesJ.size(); mj++) {
                        int64_t dist = std::abs(framesI[mi].hwTimestampUs - framesJ[mj].hwTimestampUs);
                        if (dist < bestDist) {
                            bestDist = dist;
                            bestIdx  = mj;
                        }
                    }

                    if (bestDist < hwThresholdUs) {
                        int64_t diff = framesI[mi].sysTimestampUs - framesJ[bestIdx].sysTimestampUs;
                        pairDiffs_[key].push_back(diff);
                    } else {
                        missedCount_++;
                    }
                }

                auto snI = devices_[i]->getDeviceInfo()->serialNumber();
                auto snJ = devices_[j]->getDeviceInfo()->serialNumber();
                std::cout << "Pair (" << i << ":" << snI << " vs " << j << ":" << snJ
                          << "): matched=" << pairDiffs_[key].size()
                          << "  missed=" << missedCount_ << std::endl;
            }
        }

        // Total matched count
        int totalMatched = 0;
        for (auto& kv : pairDiffs_) totalMatched += static_cast<int>(kv.second.size());
        std::cout << "\nTotal matched pairs: " << totalMatched << std::endl;
        std::cout << "Total missed (unmatched): " << missedCount_ << std::endl;
    }
```

- [ ] **Step 2: Commit**

```bash
git add timestamp_sync_check.cpp
git commit -m "feat: implement pairwise frame matching across all camera pairs

- O(N^2) pairs, for each pair match by closest hw timestamp
- hwThreshold (default 500us) gates valid pairing
- Store sys timestamp diffs per pair in pairDiffs_ map

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 4: 实现 Step 6 — Pairwise 统计 + 全局汇总 + CSV

**Files:**
- Modify: `timestamp_sync_check.cpp`

**Interfaces:**
- Consumes: Task 3 的 `pairDiffs_`, `missedCount_`
- Produces: `bool printStatistics()` — 输出每对统计、全局汇总、直方图、CSV，返回全局 PASS/FAIL

- [ ] **Step 1: 在 matchAndComputeDiffs 之后添加 printStatistics 方法**

```cpp
    // ---- Step 6: Statistics output (per-pair + global + histogram + CSV) ----
    bool printStatistics() {
        int deviceCount = static_cast<int>(devices_.size());
        bool globalPass = true;
        int64_t globalAbsMax = 0;
        int worstI = 0, worstJ = 0;

        // Prepare CSV output if requested
        std::ofstream csvFile;
        if (!csvPath.empty()) {
            csvFile.open(csvPath);
            csvFile << "device_i,device_j,diff_us" << std::endl;
        }

        std::cout << "\n==============================================" << std::endl;
        std::cout << "  Pairwise System Timestamp Sync Statistics" << std::endl;
        std::cout << "  (software end-to-end precision)" << std::endl;
        std::cout << "==============================================" << std::endl;

        for (int i = 0; i < deviceCount; i++) {
            for (int j = i + 1; j < deviceCount; j++) {
                auto key = std::make_pair(i, j);
                auto& diffs = pairDiffs_[key];

                if (diffs.empty()) {
                    std::cout << "\nDevice " << i << " vs Device " << j << ": NO MATCHED PAIRS" << std::endl;
                    globalPass = false;
                    continue;
                }

                // Sort for min/max
                std::sort(diffs.begin(), diffs.end());
                int64_t minDiff = diffs.front();
                int64_t maxDiff = diffs.back();
                int64_t absMax  = std::max(std::abs(minDiff), std::abs(maxDiff));

                // Mean
                double sum = 0.0;
                for (auto d : diffs) sum += static_cast<double>(d);
                double mean = sum / diffs.size();

                // Stddev
                double sqSum = 0.0;
                for (auto d : diffs) {
                    double delta = static_cast<double>(d) - mean;
                    sqSum += delta * delta;
                }
                double stddev = std::sqrt(sqSum / diffs.size());

                bool pairPass = (absMax < syncThresholdUs);

                auto snI = devices_[i]->getDeviceInfo()->serialNumber();
                auto snJ = devices_[j]->getDeviceInfo()->serialNumber();

                std::cout << "\n----------------------------------------------" << std::endl;
                std::cout << "Device " << i << " (SN=" << snI << ") vs Device "
                          << j << " (SN=" << snJ << ")" << std::endl;
                std::cout << "  Pair count:   " << diffs.size() << std::endl;
                std::cout << "  Min diff:     " << std::fixed << std::setprecision(1)
                          << (minDiff / 1000.0) << " us  (" << minDiff << " us)" << std::endl;
                std::cout << "  Max diff:     " << (maxDiff / 1000.0) << " us  (" << maxDiff << " us)" << std::endl;
                std::cout << "  Abs max:      " << (absMax / 1000.0) << " us  (" << absMax << " us)" << std::endl;
                std::cout << "  Mean diff:    " << (mean / 1000.0) << " us  (" << static_cast<int64_t>(mean) << " us)" << std::endl;
                std::cout << "  Stddev:       " << (stddev / 1000.0) << " us  (" << static_cast<int64_t>(stddev) << " us)" << std::endl;
                std::cout << "  Verdict:      " << (pairPass ? "PASS" : "FAIL")
                          << " (threshold=" << (syncThresholdUs / 1000.0) << " ms)" << std::endl;

                if (!pairPass) globalPass = false;
                if (absMax > globalAbsMax) {
                    globalAbsMax = absMax;
                    worstI = i;
                    worstJ = j;
                }

                // CSV output
                if (csvFile.is_open()) {
                    for (auto d : diffs) {
                        csvFile << i << "," << j << "," << d << std::endl;
                    }
                }
            }
        }

        // Global summary
        std::cout << "\n==============================================" << std::endl;
        std::cout << "  Global Summary" << std::endl;
        std::cout << "==============================================" << std::endl;
        auto snWorstI = devices_[worstI]->getDeviceInfo()->serialNumber();
        auto snWorstJ = devices_[worstJ]->getDeviceInfo()->serialNumber();
        std::cout << "Total pairs:       " << (deviceCount * (deviceCount - 1) / 2) << std::endl;
        std::cout << "Worst pair:        Device " << worstI << " (SN=" << snWorstI
                  << ") vs Device " << worstJ << " (SN=" << snWorstJ << ")" << std::endl;
        std::cout << "Worst abs max:     " << std::fixed << std::setprecision(1)
                  << (globalAbsMax / 1000.0) << " us  (" << globalAbsMax << " us)" << std::endl;
        std::cout << "Sync threshold:    " << (syncThresholdUs / 1000.0) << " ms" << std::endl;
        std::cout << "Global verdict:    " << (globalPass ? "PASS (< 1ms)" : "FAIL (>= 1ms)") << std::endl;
        std::cout << "==============================================" << std::endl;

        if (csvFile.is_open()) {
            csvFile.close();
            std::cout << "\nCSV exported to: " << csvPath << std::endl;
        }

        // Histogram for worst pair (or all pairs if --histogram-all)
        std::vector<std::pair<int,int>> histogramPairs;
        if (histogramAll) {
            for (int i = 0; i < deviceCount; i++)
                for (int j = i + 1; j < deviceCount; j++)
                    histogramPairs.push_back({i, j});
        } else {
            histogramPairs.push_back({worstI, worstJ});
        }

        for (auto& hp : histogramPairs) {
            auto key = std::make_pair(hp.first, hp.second);
            auto& diffs = pairDiffs_[key];
            if (diffs.empty()) continue;
            if (!histogramAll && diffs.size() < 2) continue;

            std::sort(diffs.begin(), diffs.end());
            int64_t dMin = diffs.front();
            int64_t dMax = diffs.back();
            double binWidth = static_cast<double>(dMax - dMin) / histogramBins;
            if (binWidth <= 0.0) continue;

            std::vector<int> bins(histogramBins, 0);
            for (auto d : diffs) {
                int idx = static_cast<int>((d - dMin) / binWidth);
                if (idx >= histogramBins) idx = histogramBins - 1;
                bins[idx]++;
            }

            auto snA = devices_[hp.first]->getDeviceInfo()->serialNumber();
            auto snB = devices_[hp.second]->getDeviceInfo()->serialNumber();
            std::cout << "\nHistogram: Device " << hp.first << " (SN=" << snA
                      << ") vs Device " << hp.second << " (SN=" << snB
                      << ") [" << histogramBins << " bins]" << std::endl;

            int maxCount = *std::max_element(bins.begin(), bins.end());
            const int barWidth = 40;
            for (int b = 0; b < histogramBins; b++) {
                double binStart = dMin + b * binWidth;
                double binEnd   = binStart + binWidth;
                int barLen = (maxCount > 0) ? (bins[b] * barWidth / maxCount) : 0;
                std::cout << "  [" << std::setw(8) << std::fixed << std::setprecision(1)
                          << (binStart / 1000.0) << " - " << std::setw(8)
                          << (binEnd / 1000.0) << " us]  "
                          << std::setw(5) << bins[b] << " |"
                          << std::string(barLen, '#') << std::endl;
            }
        }

        return globalPass;
    }
```

- [ ] **Step 2: Commit**

```bash
git add timestamp_sync_check.cpp
git commit -m "feat: pairwise statistics, global summary, CSV export, histogram

- Per-pair stats: count, min, max, mean, stddev, PASS/FAIL
- Global summary: worst pair, global verdict
- Optional CSV export (--csv=path)
- Histogram for worst pair (or all with --histogram-all)

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 5: 实现命令行参数解析 + main() + README

**Files:**
- Modify: `timestamp_sync_check.cpp`
- Create: `README_timestamp_sync_check.md`

**Interfaces:**
- Consumes: Task 4 的类完整实现
- Produces: `int main()` with argc/argv parsing, `README_timestamp_sync_check.md`

- [ ] **Step 1: 重写 main() 函数**

替换文件末尾的 `main()`（第 371-386 行）为：

```cpp
// ============================================================================
// Command-line argument parsing (zero external dependencies)
// ============================================================================
static void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [OPTIONS]\n"
              << "Options:\n"
              << "  --duration=N        Collection duration in seconds (default: 30)\n"
              << "  --hw-threshold=N    HW timestamp pairing threshold in us (default: 500)\n"
              << "  --sync-threshold=N  Sync accuracy threshold in us (default: 1000 = 1ms)\n"
              << "  --fps=N             Stream frame rate (default: 30)\n"
              << "  --width=N           Depth stream width (default: 640)\n"
              << "  --height=N          Depth stream height (default: 400)\n"
              << "  --csv=PATH          Export per-diff rows to CSV file\n"
              << "  --histogram-bins=N  Number of histogram bins (default: 10)\n"
              << "  --histogram-all     Show histogram for all pairs (default: worst only)\n"
              << "  --help              Show this help message\n"
              << std::endl;
}

static bool parseArg(const std::string& arg, const std::string& prefix, int& out) {
    if (arg.compare(0, prefix.size(), prefix) == 0) {
        std::string val = arg.substr(prefix.size());
        out = std::atoi(val.c_str());
        return true;
    }
    return false;
}

static bool parseArg(const std::string& arg, const std::string& prefix, std::string& out) {
    if (arg.compare(0, prefix.size(), prefix) == 0) {
        out = arg.substr(prefix.size());
        return true;
    }
    return false;
}

// ============================================================================
// main
// ============================================================================
int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    try {
        TimestampSyncChecker checker;

        // Parse command-line arguments
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return EXIT_SUCCESS;
            }
            if (arg == "--histogram-all") {
                checker.histogramAll = true;
                continue;
            }
            if (parseArg(arg, "--duration=",         checker.durationSec))    continue;
            if (parseArg(arg, "--hw-threshold=",     checker.hwThresholdUs))   continue;
            if (parseArg(arg, "--sync-threshold=",   checker.syncThresholdUs)) continue;
            if (parseArg(arg, "--fps=",              checker.fps))             continue;
            if (parseArg(arg, "--width=",            checker.width))           continue;
            if (parseArg(arg, "--height=",           checker.height))          continue;
            if (parseArg(arg, "--histogram-bins=",   checker.histogramBins))   continue;
            if (parseArg(arg, "--csv=",              checker.csvPath))         continue;
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }

        std::cout << "Configuration:" << std::endl;
        std::cout << "  duration=" << checker.durationSec << "s"
                  << "  hw-threshold=" << checker.hwThresholdUs << "us"
                  << "  sync-threshold=" << checker.syncThresholdUs << "us"
                  << "  fps=" << checker.fps
                  << "  resolution=" << checker.width << "x" << checker.height << std::endl;
        if (!checker.csvPath.empty()) {
            std::cout << "  csv=" << checker.csvPath << std::endl;
        }
        std::cout << "  histogram-bins=" << checker.histogramBins
                  << "  histogram-all=" << (checker.histogramAll ? "true" : "false")
                  << std::endl;

        checker.enumerateDevices();
        checker.configureSyncMode();
        checker.resetTimestampAndSyncClock();
        checker.collectFrames();
        checker.matchAndComputeDiffs();
        return checker.printStatistics() ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
```

- [ ] **Step 2: 创建 README_timestamp_sync_check.md**

```markdown
# Timestamp Sync Check

验证硬件同步的多台 Orbbec 相机在软件端到端的时间戳精度 < 1ms。

## 原理

- **硬件时间戳** (`OB_FRAME_METADATA_TYPE_TIMESTAMP`): 用于帧配对。硬件同步保证同一帧在所有相机上的硬件时间戳一致。
- **软件时间戳** (`std::chrono::system_clock::now()`): 在 SDK 回调入口立刻记录，用于对比验证。两台相机同一帧的软件时间戳差值 = 软件端到端精度。
- **Pairwise 对比**: 全矩阵配对，任意两台相机之间都做对比，输出最差情况。

## 编译

```bash
cd ob_tools
mkdir -p build && cd build
cmake .. -DOrbbecSDK_DIR=<path_to_orbbec_sdk>/lib
make -j$(nproc)
```

## 硬件连线

1. 用硬件同步线连接所有相机（菊花链或星型）
2. 所有相机通过 USB 连接到同一台主机
3. 增大 USB 缓冲区：

```bash
echo 512 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

## 使用

```bash
# 默认参数：30秒采集，30fps，640x400
./timestamp_sync_check

# 自定义参数
./timestamp_sync_check \
    --duration=60 \
    --sync-threshold=500 \
    --csv=result.csv \
    --histogram-all

# 查看帮助
./timestamp_sync_check --help
```

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--duration=N` | 30 | 采集时长（秒） |
| `--hw-threshold=N` | 500 | 硬件时间戳配对阈值（us） |
| `--sync-threshold=N` | 1000 | 软件同步精度判定阈值（us），即 <1ms |
| `--fps=N` | 30 | 采集帧率 |
| `--width=N` | 640 | 深度图宽度 |
| `--height=N` | 400 | 深度图高度 |
| `--csv=PATH` | (不导出) | CSV 文件路径 |
| `--histogram-bins=N` | 10 | 直方图 bin 数 |
| `--histogram-all` | false | 对所有 pair 画直方图 |

## 输出解读

### 每对相机统计

```
Device 0 (SN=xxx) vs Device 1 (SN=yyy)
  Pair count:   898
  Min diff:     -120.5 us
  Max diff:     350.2 us
  Abs max:      350.2 us
  Mean diff:    15.3 us
  Stddev:       45.7 us
  Verdict:      PASS (threshold=1.0 ms)
```

### 全局汇总

```
Global Summary
  Total pairs:       3
  Worst pair:        Device 0 vs Device 1
  Worst abs max:     350.2 us
  Sync threshold:    1.0 ms
  Global verdict:    PASS (< 1ms)
```

### 判定标准

- **PASS**: 所有 pair 的 |Max diff| < `--sync-threshold` (默认 1000us = 1ms)
- **FAIL**: 任意 pair 的 |Max diff| >= `--sync-threshold`

### CSV 格式

```csv
device_i,device_j,diff_us
0,1,15
0,1,-8
0,2,23
1,2,-5
...
```
```

- [ ] **Step 3: Commit**

```bash
git add timestamp_sync_check.cpp README_timestamp_sync_check.md
git commit -m "feat: add command-line parsing, main(), and README

- Manual argc/argv parsing (zero extra dependencies)
- Print configuration on startup
- README_timestamp_sync_check.md with usage and output guide

Co-Authored-By: Claude <noreply@anthropic.com>"
```
```

- [ ] **Step 4: 编译验证（如果在 Linux ARM64 环境）**

```bash
cd ob_tools/build
cmake .. -DOrbbecSDK_DIR=../OrbbecSDK_v2.8.7_202606161335_ab8672c_linux_arm64/lib
make -j$(nproc)
```

预期：编译成功，无错误无警告。

- [ ] **Step 5: 更新设计文档进度**

在 `docs/superpowers/specs/2026-07-10-pairwise-sync-check-design.md` 末尾追加：

```markdown
## 实现进度

- [x] Task 1: 数据结构重构 — master/slave → N 设备 vector
- [x] Task 2: Step 4 — N 设备采集帧
- [x] Task 3: Step 5 — 全矩阵 Pairwise 帧配对
- [x] Task 4: Step 6 — Pairwise 统计 + 全局汇总 + CSV
- [x] Task 5: 命令行参数解析 + main() + README
```

- [ ] **Step 6: Final commit**

```bash
git add docs/superpowers/specs/2026-07-10-pairwise-sync-check-design.md
git commit -m "docs: mark implementation tasks complete

Co-Authored-By: Claude <noreply@anthropic.com>"
```
```