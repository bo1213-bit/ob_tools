# Refactor timestamp_sync_check.cpp into a Class — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `timestamp_sync_check.cpp` 中 `main()` 的 6 个步骤拆分为 `TimestampSyncChecker` 类的 6 个成员方法，不改变任何业务逻辑。

**Architecture:** 单文件重构。`FrameStamp` 结构体和 `g_running`/`signalHandler` 保持在类外，其余逻辑全部移入 `TimestampSyncChecker` 类。每个步骤对应一个 public 方法，`main()` 简化为顺序调用链。

**Tech Stack:** C++11, Orbbec SDK v2, CMake

## Global Constraints

- 不改变任何业务逻辑，只做结构重组
- 不修改 CMakeLists.txt
- 不新增文件
- 保持 C++11 标准
- 异常处理不变：`ob::Error` 由 main() 统一 catch
- 信号处理保持类外：`g_running` + `signalHandler`

---

## File Structure

| 文件 | 职责 |
|------|------|
| `timestamp_sync_check.cpp` | 唯一修改文件。重构后包含：`FrameStamp` 结构体、`g_running`/`signalHandler`、`TimestampSyncChecker` 类定义、`main()` |

---

### Task 1: Refactor into TimestampSyncChecker class

**Files:**
- Modify: `timestamp_sync_check.cpp`（整个文件重写）

**Interfaces:**
- Consumes: 无（这是唯一任务）
- Produces: `TimestampSyncChecker` 类，包含以下 public 方法：
  - `void enumerateDevices()`
  - `void configureSyncMode()`
  - `void resetTimestampAndSyncClock()`
  - `void collectFrames(int durationSec)`
  - `void matchFrames()`
  - `bool printStatistics()`

**类定义和成员变量：**

```cpp
class TimestampSyncChecker {
public:
    void enumerateDevices();
    void configureSyncMode();
    void resetTimestampAndSyncClock();
    void collectFrames(int durationSec);
    void matchFrames();
    bool printStatistics();

private:
    static constexpr int kFPS   = 30;
    static constexpr int kWidth = 640;
    static constexpr int kHeight = 400;

    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device>  masterDev_;
    std::shared_ptr<ob::Device>  slaveDev_;
    std::shared_ptr<ob::Pipeline> masterPipe_;
    std::shared_ptr<ob::Pipeline> slavePipe_;

    std::queue<FrameStamp> masterQueue_;
    std::queue<FrameStamp> slaveQueue_;
    std::mutex masterMutex_;
    std::mutex slaveMutex_;

    std::vector<FrameStamp> masterFrames_;
    std::vector<FrameStamp> slaveFrames_;
    std::vector<int64_t> diffs_;
    int missedCount_ = 0;
};
```

- [ ] **Step 1: 替换整个文件内容**

将 `timestamp_sync_check.cpp` 的全部内容替换为以下代码。每段代码来自原始 `main()` 的对应部分，逐行搬运，业务逻辑不变。

```cpp
// timestamp_sync_check.cpp
//
// Measures timestamp difference between two hardware-synced Orbbec cameras
// to verify sync accuracy < 1ms.
//
// Step 1: Enumerate devices and print device info
// Step 2: Configure master (PRIMARY) and slave (SECONDARY) sync mode
// Step 3: Timestamp reset + device clock sync
// Step 4: Start pipelines, collect frames in callbacks
// Step 5: Match frames by system timestamp, compute hw timestamp diff
// Step 6: Statistics output (min/max/mean/stddev/histogram/verdict)

#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// Each frame's timestamp info we store in the queue
// ============================================================================
struct FrameStamp {
    int64_t hwTimestampUs;   // hardware timestamp (getTimeStampUs)
    int64_t sysTimestampUs;  // system timestamp (getSystemTimeStampUs)
    int64_t frameNumber;     // frame sequence number
};

// ============================================================================
// Global state for signal handling (Ctrl+C during collection)
// ============================================================================
static std::atomic<bool> g_running{true};

static void signalHandler(int /*signum*/) {
    g_running = false;
    std::cout << "\n[INFO] Received signal, shutting down..." << std::endl;
}

// ============================================================================
// TimestampSyncChecker — orchestrates the 6-step sync check
// ============================================================================
class TimestampSyncChecker {
public:
    // ---- Step 1: Enumerate devices ----
    void enumerateDevices() {
        // Create context
        context_ = std::make_shared<ob::Context>();

        // Query device list
        auto devList = context_->queryDeviceList();
        int devCount = devList->deviceCount();

        std::cout << "Found " << devCount << " device(s)" << std::endl;

        // Print info for each device
        for (int i = 0; i < devCount; i++) {
            auto dev  = devList->getDevice(i);
            auto info = dev->getDeviceInfo();

            std::cout << "Device " << i << ": "
                      << "SN=" << info->serialNumber()
                      << "  Name=" << info->getName()
                      << "  PID=" << info->getPid()
                      << "  VID=" << info->getVid()
                      << std::endl;

            // Check supported sync modes
            auto syncBitmap = dev->getSupportedMultiDeviceSyncModeBitmap();
            std::cout << "  Supported sync modes: 0x" << std::hex << syncBitmap << std::dec << std::endl;
        }

        if (devCount < 2) {
            throw std::runtime_error("Need at least 2 devices for sync check!");
        }

        // Assign master and slave
        masterDev_ = devList->getDevice(0);
        slaveDev_  = devList->getDevice(1);
        std::cout << "\nMaster: " << masterDev_->getDeviceInfo()->serialNumber() << std::endl;
        std::cout << "Slave:  " << slaveDev_->getDeviceInfo()->serialNumber() << std::endl;
    }

    // ---- Step 2: Configure sync mode ----
    void configureSyncMode() {
        // --- Master: PRIMARY mode, trigger output enabled ---
        OBMultiDeviceSyncConfig masterCfg = masterDev_->getMultiDeviceSyncConfig();
        masterCfg.syncMode             = OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
        masterCfg.triggerOutEnable     = true;
        masterCfg.triggerOutDelayUs    = 0;
        masterCfg.depthDelayUs         = 0;
        masterCfg.colorDelayUs         = 0;
        masterCfg.trigger2ImageDelayUs = 0;
        masterCfg.framesPerTrigger     = 1;
        masterDev_->setMultiDeviceSyncConfig(masterCfg);
        std::cout << "Master configured: PRIMARY, triggerOut=true" << std::endl;

        // --- Slave: SECONDARY mode, trigger output disabled ---
        OBMultiDeviceSyncConfig slaveCfg = slaveDev_->getMultiDeviceSyncConfig();
        slaveCfg.syncMode             = OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
        slaveCfg.triggerOutEnable     = false;
        slaveCfg.triggerOutDelayUs    = 0;
        slaveCfg.depthDelayUs         = 0;
        slaveCfg.colorDelayUs         = 0;
        slaveCfg.trigger2ImageDelayUs = 0;
        slaveCfg.framesPerTrigger     = 1;
        slaveDev_->setMultiDeviceSyncConfig(slaveCfg);
        std::cout << "Slave configured: SECONDARY, triggerOut=false" << std::endl;

        // Verify the config was applied
        {
            auto checkMaster = masterDev_->getMultiDeviceSyncConfig();
            auto checkSlave  = slaveDev_->getMultiDeviceSyncConfig();
            std::cout << "\nVerify config:" << std::endl;
            std::cout << "  Master syncMode=" << checkMaster.syncMode
                      << "  triggerOut=" << checkMaster.triggerOutEnable << std::endl;
            std::cout << "  Slave  syncMode=" << checkSlave.syncMode
                      << "  triggerOut=" << checkSlave.triggerOutEnable << std::endl;
        }
    }

    // ---- Step 3: Timestamp reset + device clock sync ----
    void resetTimestampAndSyncClock() {
        // Send timestamp reset signal so master and slave clocks start from the same moment
        masterDev_->setBoolProperty(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, true);
        masterDev_->setIntProperty(OB_PROP_TIMER_RESET_DELAY_US_INT, 20);
        masterDev_->setBoolProperty(OB_PROP_TIMER_RESET_SIGNAL_BOOL, true);
        std::cout << "\nTimestamp reset sent (master -> slave, delay=20us)" << std::endl;

        // Enable device clock sync every 100ms to prevent long-term drift
        context_->enableDeviceClockSync(100);
        std::cout << "Device clock sync enabled (every 100ms)" << std::endl;
    }

    // ---- Step 4: Start pipelines, collect frames, stop, drain queues ----
    void collectFrames(int durationSec) {
        // --- Start master pipeline ---
        masterPipe_ = std::make_shared<ob::Pipeline>(masterDev_);
        auto masterCfg = std::make_shared<ob::Config>();
        masterCfg->enableVideoStream(OB_STREAM_DEPTH, kWidth, kHeight, kFPS, OB_FORMAT_Y16);

        masterPipe_->start(masterCfg, [this](std::shared_ptr<ob::FrameSet> frameSet) {
            auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
            if (!depthFrame) return;

            FrameStamp fs;
            fs.hwTimestampUs  = depthFrame->getTimeStampUs();
            fs.sysTimestampUs = depthFrame->getSystemTimeStampUs();
            fs.frameNumber    = depthFrame->getIndex();

            std::lock_guard<std::mutex> lock(masterMutex_);
            masterQueue_.push(fs);
        });
        std::cout << "\nMaster pipeline started (depth " << kWidth << "x" << kHeight
                  << " @ " << kFPS << "fps)" << std::endl;

        // --- Start slave pipeline ---
        slavePipe_ = std::make_shared<ob::Pipeline>(slaveDev_);
        auto slaveCfg = std::make_shared<ob::Config>();
        slaveCfg->enableVideoStream(OB_STREAM_DEPTH, kWidth, kHeight, kFPS, OB_FORMAT_Y16);

        slavePipe_->start(slaveCfg, [this](std::shared_ptr<ob::FrameSet> frameSet) {
            auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
            if (!depthFrame) return;

            FrameStamp fs;
            fs.hwTimestampUs  = depthFrame->getTimeStampUs();
            fs.sysTimestampUs = depthFrame->getSystemTimeStampUs();
            fs.frameNumber    = depthFrame->getIndex();

            std::lock_guard<std::mutex> lock(slaveMutex_);
            slaveQueue_.push(fs);
        });
        std::cout << "Slave pipeline started (depth " << kWidth << "x" << kHeight
                  << " @ " << kFPS << "fps)" << std::endl;

        // --- Collect frames ---
        std::cout << "Collecting frames for " << durationSec
                  << " seconds... (Ctrl+C to stop early)" << std::endl;
        auto startTime = std::chrono::steady_clock::now();
        while (g_running) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - startTime).count();
            if (elapsed >= durationSec) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // --- Stop pipelines ---
        slavePipe_->stop();
        masterPipe_->stop();
        std::cout << "Pipelines stopped" << std::endl;

        // --- Print how many frames were collected ---
        {
            std::lock_guard<std::mutex> lockM(masterMutex_);
            std::lock_guard<std::mutex> lockS(slaveMutex_);
            std::cout << "\nMaster frames collected: " << masterQueue_.size() << std::endl;
            std::cout << "Slave frames collected:  " << slaveQueue_.size() << std::endl;
        }

        // --- Drain queues into vectors ---
        {
            std::lock_guard<std::mutex> lockM(masterMutex_);
            std::lock_guard<std::mutex> lockS(slaveMutex_);
            while (!masterQueue_.empty()) {
                masterFrames_.push_back(masterQueue_.front());
                masterQueue_.pop();
            }
            while (!slaveQueue_.empty()) {
                slaveFrames_.push_back(slaveQueue_.front());
                slaveQueue_.pop();
            }
        }
        std::cout << "\nTransferred to vectors: master=" << masterFrames_.size()
                  << "  slave=" << slaveFrames_.size() << std::endl;
    }

    // ---- Step 5: Match frames by system timestamp ----
    void matchFrames() {
        int64_t sysThreshold = (1000000 / kFPS) * 2;  // 2 frame periods

        missedCount_ = 0;

        for (size_t m = 0; m < masterFrames_.size(); m++) {
            int64_t bestDist = INT64_MAX;
            size_t  bestIdx  = 0;

            // Find the slave frame with closest system timestamp
            for (size_t s = 0; s < slaveFrames_.size(); s++) {
                int64_t dist = std::abs(masterFrames_[m].sysTimestampUs - slaveFrames_[s].sysTimestampUs);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx  = s;
                }
            }

            // Only accept if within threshold
            if (bestDist < sysThreshold) {
                int64_t diff = masterFrames_[m].hwTimestampUs - slaveFrames_[bestIdx].hwTimestampUs;
                diffs_.push_back(diff);
            } else {
                missedCount_++;
            }
        }

        std::cout << "Matched pairs: " << diffs_.size() << std::endl;
        std::cout << "Missed (unmatched): " << missedCount_ << std::endl;
    }

    // ---- Step 6: Statistics output ----
    bool printStatistics() {
        if (diffs_.empty()) {
            std::cerr << "ERROR: No matched pairs - cannot compute statistics." << std::endl;
            return false;
        }

        // 6a. Sort to get min / max
        std::sort(diffs_.begin(), diffs_.end());

        int64_t minDiff = diffs_.front();
        int64_t maxDiff = diffs_.back();
        int64_t absMax  = std::max(std::abs(minDiff), std::abs(maxDiff));

        // 6b. Compute mean
        double sum = 0.0;
        for (auto d : diffs_) {
            sum += static_cast<double>(d);
        }
        double mean = sum / diffs_.size();

        // 6c. Compute stddev
        double sqSum = 0.0;
        for (auto d : diffs_) {
            double delta = static_cast<double>(d) - mean;
            sqSum += delta * delta;
        }
        double stddev = std::sqrt(sqSum / diffs_.size());

        // 6d. Output statistics
        std::cout << "\n==============================================" << std::endl;
        std::cout << "  Timestamp Sync Statistics" << std::endl;
        std::cout << "==============================================" << std::endl;
        std::cout << "Pair count:    " << diffs_.size() << std::endl;
        std::cout << "Missed:        " << missedCount_ << std::endl;
        std::cout << "Min diff:      " << std::fixed << std::setprecision(1)
                  << (minDiff / 1000.0) << " us  (" << minDiff << " us)" << std::endl;
        std::cout << "Max diff:      " << (maxDiff / 1000.0) << " us  (" << maxDiff << " us)" << std::endl;
        std::cout << "Abs max:       " << (absMax / 1000.0) << " us  (" << absMax << " us)" << std::endl;
        std::cout << "Mean diff:     " << (mean / 1000.0) << " us  (" << static_cast<int64_t>(mean) << " us)" << std::endl;
        std::cout << "Stddev:        " << (stddev / 1000.0) << " us  (" << static_cast<int64_t>(stddev) << " us)" << std::endl;
        std::cout << "----------------------------------------------" << std::endl;

        // 6e. Verdict: is sync accuracy < 1ms?
        const int64_t kThresholdUs = 1000;  // 1ms
        bool pass = (absMax < kThresholdUs);
        std::cout << "Threshold:     " << (kThresholdUs / 1000.0) << " ms" << std::endl;
        std::cout << "Verdict:       " << (pass ? "PASS (< 1ms)" : "FAIL (>= 1ms)") << std::endl;
        std::cout << "==============================================" << std::endl;

        // 6f. Optional histogram (10 bins)
        const int numBins = 10;
        double binWidth = static_cast<double>(maxDiff - minDiff) / numBins;
        if (binWidth > 0.0) {
            std::vector<int> bins(numBins, 0);
            for (auto d : diffs_) {
                int idx = static_cast<int>((d - minDiff) / binWidth);
                if (idx >= numBins) idx = numBins - 1;  // clamp maxDiff to last bin
                bins[idx]++;
            }

            std::cout << "\nHistogram (" << numBins << " bins):" << std::endl;
            int maxCount = *std::max_element(bins.begin(), bins.end());
            const int barWidth = 40;
            for (int i = 0; i < numBins; i++) {
                double binStart = minDiff + i * binWidth;
                double binEnd   = binStart + binWidth;
                int barLen = (maxCount > 0) ? (bins[i] * barWidth / maxCount) : 0;
                std::cout << "  [" << std::setw(8) << std::fixed << std::setprecision(1)
                          << (binStart / 1000.0) << " - " << std::setw(8)
                          << (binEnd / 1000.0) << " us]  "
                          << std::setw(5) << bins[i] << " |"
                          << std::string(barLen, '#') << std::endl;
            }
        }

        return pass;
    }

private:
    static constexpr int kFPS   = 30;
    static constexpr int kWidth = 640;
    static constexpr int kHeight = 400;

    std::shared_ptr<ob::Context>   context_;
    std::shared_ptr<ob::Device>    masterDev_;
    std::shared_ptr<ob::Device>    slaveDev_;
    std::shared_ptr<ob::Pipeline>  masterPipe_;
    std::shared_ptr<ob::Pipeline>  slavePipe_;

    std::queue<FrameStamp> masterQueue_;
    std::queue<FrameStamp> slaveQueue_;
    std::mutex masterMutex_;
    std::mutex slaveMutex_;

    std::vector<FrameStamp> masterFrames_;
    std::vector<FrameStamp> slaveFrames_;
    std::vector<int64_t>    diffs_;
    int missedCount_ = 0;
};

// ============================================================================
// main
// ============================================================================
int main() {
    signal(SIGINT, signalHandler);

    try {
        TimestampSyncChecker checker;
        checker.enumerateDevices();
        checker.configureSyncMode();
        checker.resetTimestampAndSyncClock();
        checker.collectFrames(30);
        checker.matchFrames();
        return checker.printStatistics() ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
```

- [ ] **Step 2: 验证文件内容正确**

确认文件开头注释、`#include` 列表、`FrameStamp` 结构体、`g_running`/`signalHandler`、类定义、`main()` 全部就位。确认没有遗漏的代码段。

- [ ] **Step 3: 编译验证（如果在 Linux ARM64 环境）**

```bash
cd ob_tools
mkdir -p build && cd build
cmake .. -DOrbbecSDK_DIR=../OrbbecSDK_v2.8.7_202606161335_ab8672c_linux_arm64/lib
make -j$(nproc)
```

预期：编译成功，无错误无警告。

- [ ] **Step 4: 更新 DESIGN.md 进度**

将 `DESIGN.md` 中当前进度部分更新为：
```markdown
## 当前进度

- [x] Step 1-6: 已重构为 TimestampSyncChecker 类
- [x] 类封装：6 个方法对应 6 个步骤

## 待完成

- fps_quality_test.cpp：不同帧率下图像质量测试工具
```

- [ ] **Step 5: Commit**

```bash
git add timestamp_sync_check.cpp DESIGN.md docs/
git commit -m "refactor: extract main() steps into TimestampSyncChecker class

6 steps become 6 member methods, main() is now a simple call chain.
No behavioral changes — pure structural refactoring.

Co-Authored-By: Claude <noreply@anthropic.com>"
```