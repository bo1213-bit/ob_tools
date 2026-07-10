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
    int testRunCount     = 0;

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
                          << "): matched=" << pairDiffs_[key].size() << std::endl;
            }
        }

        // Total matched count
        int totalMatched = 0;
        for (auto& kv : pairDiffs_) totalMatched += static_cast<int>(kv.second.size());
        std::cout << "\nTotal matched pairs: " << totalMatched << std::endl;
        std::cout << "Total missed (unmatched): " << missedCount_ << std::endl;
    }

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