// sync_analyzer.cpp
// 模块2: 时间戳对比分析 — 三类对比 + 统计 + CSV 导出

#include "sync_analyzer.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace {

struct SortedFrame {
    const FrameStamp* frame;
    size_t originalIndex;
};

std::vector<SortedFrame> sortByGlobalTimestamp(const std::vector<FrameStamp>& frames) {
    std::vector<SortedFrame> sorted;
    sorted.reserve(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        sorted.push_back({&frames[i], i});
    }
    std::sort(sorted.begin(), sorted.end(), [](const SortedFrame& lhs, const SortedFrame& rhs) {
        return lhs.frame->globalTimestampUs < rhs.frame->globalTimestampUs;
    });
    return sorted;
}

// Equivalent to DeviceFrames.nearest_from() in the official analyze_sync.py.
// Only frames at or after startIndex can be selected, so a selected frame cannot
// be reused by a later group. Equal-distance candidates prefer the earlier one.
size_t nearestFrom(const std::vector<SortedFrame>& frames,
                   int64_t targetTimestampUs,
                   size_t startIndex,
                   int64_t maxDiffUs) {
    size_t lastBelow = frames.size();
    size_t index = startIndex;

    while (index < frames.size() && frames[index].frame->globalTimestampUs < targetTimestampUs) {
        if (frames[index].frame->globalTimestampUs >= targetTimestampUs - maxDiffUs) {
            lastBelow = index;
        }
        ++index;
    }

    const bool hasBelow = lastBelow != frames.size();
    const bool hasAbove = index < frames.size() &&
                          frames[index].frame->globalTimestampUs <= targetTimestampUs + maxDiffUs;
    if (!hasBelow && !hasAbove) {
        return frames.size();
    }
    if (!hasBelow) {
        return index;
    }
    if (!hasAbove) {
        return lastBelow;
    }

    const int64_t belowDiff = targetTimestampUs - frames[lastBelow].frame->globalTimestampUs;
    const int64_t aboveDiff = frames[index].frame->globalTimestampUs - targetTimestampUs;
    return belowDiff <= aboveDiff ? lastBelow : index;
}

}  // namespace

std::tuple<std::vector<int64_t>, std::vector<int64_t>, std::vector<int64_t>, std::vector<int64_t>, std::vector<size_t>, std::vector<size_t>>
SyncAnalyzer::matchAndDiff(
    const std::vector<FrameStamp>& a,
    const std::vector<FrameStamp>& b,
    int64_t matchTolUs)
{
    std::vector<int64_t> hwDiffs, globalDiffs, sysDiffs, timestamps;
    std::vector<size_t> aIndices, bIndices;

    if (a.empty() || b.empty()) {
        return {hwDiffs, globalDiffs, sysDiffs, timestamps, aIndices, bIndices};
    }

    const auto sortedA = sortByGlobalTimestamp(a);
    const auto sortedB = sortByGlobalTimestamp(b);
    size_t bCursor = 0;

    // Match chronologically by global timestamp. A successful match consumes the
    // selected b frame, mirroring the official analyzer's cursor-based grouping.
    for (const auto& aFrame : sortedA) {
        const size_t bIndex = nearestFrom(sortedB, aFrame.frame->globalTimestampUs, bCursor, matchTolUs);
        if (bIndex == sortedB.size()) {
            continue;
        }

        const auto& bFrame = sortedB[bIndex];
        globalDiffs.push_back(aFrame.frame->globalTimestampUs - bFrame.frame->globalTimestampUs);
        hwDiffs.push_back(aFrame.frame->hwTimestampUs - bFrame.frame->hwTimestampUs);
        sysDiffs.push_back(aFrame.frame->sysTimestampUs - bFrame.frame->sysTimestampUs);
        timestamps.push_back(aFrame.frame->sysTimestampUs);
        aIndices.push_back(aFrame.originalIndex);
        bIndices.push_back(bFrame.originalIndex);
        bCursor = bIndex + 1;
    }

    return {hwDiffs, globalDiffs, sysDiffs, timestamps, aIndices, bIndices};
}

SyncAnalyzer::MultiDeviceDiffs SyncAnalyzer::multiDeviceMatch(
    const std::vector<std::vector<FrameStamp>>& allDevFrames,
    int64_t matchTolUs,
    int refDevOverride)
{
    MultiDeviceDiffs result;

    std::vector<int> validDevs;
    for (int i = 0; i < static_cast<int>(allDevFrames.size()); ++i) {
        if (!allDevFrames[i].empty()) {
            validDevs.push_back(i);
        }
    }
    if (validDevs.size() < 2) {
        return result;
    }

    // 335Lg remains the required reference when supplied. The fallback preserves
    // the existing behavior for deployments where it is not detected.
    int refDev;
    if (refDevOverride >= 0 &&
        std::find(validDevs.begin(), validDevs.end(), refDevOverride) != validDevs.end()) {
        refDev = refDevOverride;
    } else {
        refDev = validDevs.front();
        for (int device : validDevs) {
            if (allDevFrames[device].size() < allDevFrames[refDev].size()) {
                refDev = device;
            }
        }
    }

    std::vector<std::vector<SortedFrame>> sortedFrames(allDevFrames.size());
    std::vector<size_t> cursors(allDevFrames.size(), 0);
    for (int device : validDevs) {
        sortedFrames[device] = sortByGlobalTimestamp(allDevFrames[device]);
    }

    // This keeps the required 335Lg anchor while adopting the official
    // analyze_sync.py cursor rule: a frame can only match once, and matches are
    // searched from each device's unconsumed cursor forward.
    const auto& referenceFrames = sortedFrames[refDev];
    while (cursors[refDev] < referenceFrames.size()) {
        const auto& reference = referenceFrames[cursors[refDev]];
        std::vector<size_t> selected = cursors;
        bool complete = true;

        for (int device : validDevs) {
            if (device == refDev) {
                continue;
            }

            const size_t matchIndex = nearestFrom(
                sortedFrames[device], reference.frame->globalTimestampUs,
                cursors[device], matchTolUs);
            if (matchIndex == sortedFrames[device].size()) {
                complete = false;
                break;
            }
            selected[device] = matchIndex;
        }

        if (!complete) {
            // The fixed 335Lg anchor cannot form a complete group. Do not consume
            // frames from the other devices; a later 335Lg frame may still match.
            ++cursors[refDev];
            continue;
        }

        std::vector<int64_t> groupGlobal;
        std::vector<int64_t> groupHw;
        groupGlobal.reserve(validDevs.size());
        groupHw.reserve(validDevs.size());
        for (int device : validDevs) {
            const auto* frame = sortedFrames[device][selected[device]].frame;
            groupGlobal.push_back(frame->globalTimestampUs);
            groupHw.push_back(frame->hwTimestampUs);
        }

        auto [globalMin, globalMax] = std::minmax_element(groupGlobal.begin(), groupGlobal.end());
        result.global.push_back(*globalMax - *globalMin);
        auto [hwMin, hwMax] = std::minmax_element(groupHw.begin(), groupHw.end());
        result.hw.push_back(*hwMax - *hwMin);

        for (int device : validDevs) {
            cursors[device] = selected[device] + 1;
        }
    }

    return result;
}

SyncAnalyzer::PairStats
SyncAnalyzer::computeStats(
    int devI, int devJ,
    StreamType st, bool isCrossStream,
    const std::vector<int64_t>& hwDiffs,
    const std::vector<int64_t>& globalDiffs,
    const std::vector<int64_t>& sysDiffs)
{
    PairStats s;
    s.deviceI       = devI;
    s.deviceJ       = devJ;
    s.streamType    = st;
    s.isCrossStream = isCrossStream;
    s.pairCount     = static_cast<int>(hwDiffs.size());
    s.hwMinUs = s.hwMaxUs = 0;
    s.hwMeanUs = s.hwStddevUs = 0.0;
    s.globalMinUs = s.globalMaxUs = 0;
    s.globalMeanUs = s.globalStddevUs = 0.0;
    s.sysMinUs = s.sysMaxUs = 0;
    s.sysMeanUs = s.sysStddevUs = 0.0;

    if (hwDiffs.empty()) return s;

    auto calc = [](const std::vector<int64_t>& diffs, int64_t& minVal, int64_t& maxVal, double& mean, double& stddev) {
        std::vector<int64_t> sorted = diffs;
        std::sort(sorted.begin(), sorted.end());
        minVal = sorted.front();
        maxVal = sorted.back();

        double sum = 0.0;
        for (auto d : diffs) sum += static_cast<double>(d);
        mean = sum / diffs.size();

        double sqSum = 0.0;
        for (auto d : diffs) {
            double delta = static_cast<double>(d) - mean;
            sqSum += delta * delta;
        }
        stddev = std::sqrt(sqSum / diffs.size());
    };

    calc(hwDiffs, s.hwMinUs, s.hwMaxUs, s.hwMeanUs, s.hwStddevUs);
    calc(globalDiffs, s.globalMinUs, s.globalMaxUs, s.globalMeanUs, s.globalStddevUs);
    calc(sysDiffs, s.sysMinUs, s.sysMaxUs, s.sysMeanUs, s.sysStddevUs);

    return s;
}

void SyncAnalyzer::run(
    const std::vector<std::vector<std::vector<FrameStamp>>>& frames,
    const std::vector<std::shared_ptr<ob::Device>>& devices,
    const Config& cfg)
{
    int deviceCount = static_cast<int>(frames.size());
    const int DEPTH_IDX = static_cast<int>(StreamType::DEPTH);
    const int COLOR_IDX = static_cast<int>(StreamType::COLOR);

    // 配对容差 = 半帧间隔 (与官方 half_gap_us = 1e6/fps/2 一致)
    int64_t matchTolUs = static_cast<int64_t>(std::llround(1000000.0 / cfg.fps / 2.0));

    // 检测基准设备: 优先找名称含 "335Lg" 的 (或 PID==2059), 作为多设备匹配的基准
    int refDevOverride = -1;
    multiRefName_ = "(fewest-frames)";
    for (size_t i = 0; i < devices.size(); i++) {
        auto info = devices[i]->getDeviceInfo();
        std::string name = info->getName() ? info->getName() : "";
        int pid = info->getPid();
        if (name.find("335Lg") != std::string::npos || name.find("335") != std::string::npos || pid == 2059) {
            refDevOverride = static_cast<int>(i);
            multiRefName_ = name.empty() ? std::to_string(i) : name;
            break;
        }
    }

    // 1. 同设备跨流: Depth vs Color (匹配容差=半帧间隔, 按全局 global 匹配; hw/global/sys 三种差值都报告)
    for (int i = 0; i < deviceCount; i++) {
        if (frames[i][DEPTH_IDX].empty() || frames[i][COLOR_IDX].empty()) continue;

        auto [hwDiffs, globalDiffs, sysDiffs, timestamps, aIdx, bIdx] = matchAndDiff(
            frames[i][DEPTH_IDX], frames[i][COLOR_IDX], matchTolUs);

        auto stats = computeStats(i, i, StreamType::DEPTH, true, hwDiffs, globalDiffs, sysDiffs);
        crossStreamStats_.push_back(stats);

        for (size_t k = 0; k < hwDiffs.size(); k++) {
            allDiffs_.push_back({"cross_stream", i, i, "depth+color", hwDiffs[k], globalDiffs[k], sysDiffs[k], timestamps[k]});
        }
    }

    // 2. 跨设备同流 Depth (匹配容差=半帧间隔, 按全局 global 匹配)
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][DEPTH_IDX].empty() || frames[j][DEPTH_IDX].empty()) continue;

            auto [hwDiffs, globalDiffs, sysDiffs, timestamps, aIdx, bIdx] = matchAndDiff(
                frames[i][DEPTH_IDX], frames[j][DEPTH_IDX], matchTolUs);

            auto stats = computeStats(i, j, StreamType::DEPTH, false, hwDiffs, globalDiffs, sysDiffs);
            crossDeviceDepthStats_.push_back(stats);

            for (size_t k = 0; k < hwDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "depth", hwDiffs[k], globalDiffs[k], sysDiffs[k], timestamps[k]});
            }
        }
    }

    // 3. 跨设备同流 Color (匹配容差=半帧间隔, 按全局 global 匹配)
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][COLOR_IDX].empty() || frames[j][COLOR_IDX].empty()) continue;

            auto [hwDiffs, globalDiffs, sysDiffs, timestamps, aIdx, bIdx] = matchAndDiff(
                frames[i][COLOR_IDX], frames[j][COLOR_IDX], matchTolUs);

            auto stats = computeStats(i, j, StreamType::COLOR, false, hwDiffs, globalDiffs, sysDiffs);
            crossDeviceColorStats_.push_back(stats);

            for (size_t k = 0; k < hwDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "color", hwDiffs[k], globalDiffs[k], sysDiffs[k], timestamps[k]});
            }
        }
    }

    // 4. 多设备同步精度: 所有设备同一流类型, 以基准设备为准匹配, 后用 max(global)-min(global)
    {
        auto calcMdStats = [&](const std::vector<int64_t>& diffs, StreamType st, int devCount) -> MultiDeviceStats {
            MultiDeviceStats s;
            s.streamType = st;
            s.deviceCount = devCount;
            s.matchCount = static_cast<int>(diffs.size());
            s.abnormalThresholdUs = cfg.globalThresholdUs;
            s.abnormalCount = 0;
            if (diffs.empty()) return s;
            auto [minIt, maxIt] = std::minmax_element(diffs.begin(), diffs.end());
            s.globalMinUs = *minIt;
            s.globalMaxUs = *maxIt;
            double sum = 0.0;
            for (auto d : diffs) {
                sum += static_cast<double>(d);
                if (d >= cfg.globalThresholdUs) s.abnormalCount++;
            }
            s.globalMeanUs = sum / diffs.size();
            double sqSum = 0.0;
            for (auto d : diffs) {
                double delta = static_cast<double>(d) - s.globalMeanUs;
                sqSum += delta * delta;
            }
            s.globalStddevUs = std::sqrt(sqSum / diffs.size());
            return s;
        };

        // Depth
        std::vector<std::vector<FrameStamp>> depthFrames(deviceCount);
        for (int i = 0; i < deviceCount; i++) {
            depthFrames[i] = frames[i][DEPTH_IDX];
        }
        multiDeviceDepthDiffs_ = multiDeviceMatch(depthFrames, matchTolUs, refDevOverride);

        // 同步精度用全局 global 极差度量 (官方 time base = global; hw 仅参考)
        multiDeviceDepthStats_ = calcMdStats(multiDeviceDepthDiffs_.global, StreamType::DEPTH, deviceCount);

        for (size_t k = 0; k < multiDeviceDepthDiffs_.global.size(); k++) {
            allDiffs_.push_back({"multi_device", 0, 0, "depth_all",
                                 multiDeviceDepthDiffs_.hw[k], multiDeviceDepthDiffs_.global[k], 0, 0});
        }

        // Color
        std::vector<std::vector<FrameStamp>> colorFrames(deviceCount);
        for (int i = 0; i < deviceCount; i++) {
            colorFrames[i] = frames[i][COLOR_IDX];
        }
        multiDeviceColorDiffs_ = multiDeviceMatch(colorFrames, matchTolUs, refDevOverride);
        multiDeviceColorStats_ = calcMdStats(multiDeviceColorDiffs_.global, StreamType::COLOR, deviceCount);

        for (size_t k = 0; k < multiDeviceColorDiffs_.global.size(); k++) {
            allDiffs_.push_back({"multi_device", 0, 0, "color_all",
                                 multiDeviceColorDiffs_.hw[k], multiDeviceColorDiffs_.global[k], 0, 0});
        }
    }
}

const std::vector<SyncAnalyzer::PairStats>& SyncAnalyzer::getCrossStreamStats() const {
    return crossStreamStats_;
}
const std::vector<SyncAnalyzer::PairStats>& SyncAnalyzer::getCrossDeviceDepthStats() const {
    return crossDeviceDepthStats_;
}
const std::vector<SyncAnalyzer::PairStats>& SyncAnalyzer::getCrossDeviceColorStats() const {
    return crossDeviceColorStats_;
}
const SyncAnalyzer::MultiDeviceStats& SyncAnalyzer::getMultiDeviceDepthStats() const {
    return multiDeviceDepthStats_;
}
const SyncAnalyzer::MultiDeviceStats& SyncAnalyzer::getMultiDeviceColorStats() const {
    return multiDeviceColorStats_;
}

void SyncAnalyzer::printOneStats(const PairStats& s) const {
    if (s.pairCount == 0) {
        std::cout << "  Pair count: 0 (no matches within threshold)" << std::endl;
        return;
    }
    std::cout << "  Global Timestamp Diff (sync precision, official time base):" << std::endl;
    std::cout << "    Min=" << s.globalMinUs << "us  Max=" << s.globalMaxUs << "us"
              << "  Mean=" << std::fixed << std::setprecision(1) << s.globalMeanUs
              << "us  Stddev=" << s.globalStddevUs << "us" << std::endl;
    std::cout << "  Device (hw) Timestamp Diff (ref only):" << std::endl;
    std::cout << "    Min=" << s.hwMinUs << "us  Max=" << s.hwMaxUs << "us"
              << "  Mean=" << std::fixed << std::setprecision(1) << s.hwMeanUs
              << "us  Stddev=" << s.hwStddevUs << "us" << std::endl;
    std::cout << "  System Timestamp Diff:" << std::endl;
    std::cout << "    Min=" << s.sysMinUs << "us  Max=" << s.sysMaxUs << "us"
              << "  Mean=" << std::fixed << std::setprecision(1) << s.sysMeanUs
              << "us  Stddev=" << s.sysStddevUs << "us" << std::endl;
}

void SyncAnalyzer::printReport() const {
    std::cout << "\n==============================================" << std::endl;
    std::cout << "  Timestamp Sync Analysis Report" << std::endl;
    std::cout << "==============================================" << std::endl;

    std::cout << "\n--- 1. Cross-Stream (Depth vs Color) ---" << std::endl;
    if (crossStreamStats_.empty()) {
        std::cout << "  (no data)" << std::endl;
    } else {
        for (auto& s : crossStreamStats_) {
            std::cout << "Device " << s.deviceI << ":" << std::endl;
            printOneStats(s);
        }
    }

    std::cout << "\n--- 2. Cross-Device Depth ---" << std::endl;
    if (crossDeviceDepthStats_.empty()) {
        std::cout << "  (no data)" << std::endl;
    } else {
        for (auto& s : crossDeviceDepthStats_) {
            std::cout << "Device " << s.deviceI << " vs Device " << s.deviceJ << ":" << std::endl;
            printOneStats(s);
        }
    }

    std::cout << "\n--- 3. Cross-Device Color ---" << std::endl;
    if (crossDeviceColorStats_.empty()) {
        std::cout << "  (no data)" << std::endl;
    } else {
        for (auto& s : crossDeviceColorStats_) {
            std::cout << "Device " << s.deviceI << " vs Device " << s.deviceJ << ":" << std::endl;
            printOneStats(s);
        }
    }

    std::cout << "\n--- 4. Multi-Device Sync (all cameras together) ---" << std::endl;
    std::cout << "  Reference device: " << multiRefName_ << std::endl;
    {
        auto printMd = [](const std::string& label, const MultiDeviceStats& s) {
            std::cout << "  " << label << " (" << s.deviceCount << " devices):" << std::endl;
            std::cout << "    Match groups: " << s.matchCount << std::endl;
            if (s.matchCount > 0) {
                std::cout << "    max(global)-min(global): Min=" << s.globalMinUs << "us  Max=" << s.globalMaxUs << "us"
                          << "  Mean=" << std::fixed << std::setprecision(1) << s.globalMeanUs
                          << "us  Stddev=" << s.globalStddevUs << "us" << std::endl;
                double abnormalPct = 100.0 * s.abnormalCount / s.matchCount;
                std::cout << "    Abnormal (>= " << s.abnormalThresholdUs << "us): "
                          << s.abnormalCount << " / " << s.matchCount
                          << " (" << std::fixed << std::setprecision(2) << abnormalPct << "%)" << std::endl;
            }
        };
        printMd("Depth", multiDeviceDepthStats_);
        printMd("Color", multiDeviceColorStats_);
    }

    std::cout << "==============================================" << std::endl;
}

void SyncAnalyzer::exportCSV(const std::string& path) const {
    std::ofstream csvFile(path);
    if (!csvFile.is_open()) {
        std::cerr << "Cannot open CSV: " << path << std::endl;
        return;
    }

    csvFile << "comparison_type,device_i,device_j,stream,hw_diff_us,global_diff_us,sys_diff_us,timestamp_us" << std::endl;

    for (auto& d : allDiffs_) {
        csvFile << d.comparisonType << ","
                << d.deviceI << ","
                << d.deviceJ << ","
                << d.streamLabel << ","
                << d.hwDiffUs << ","
                << d.globalDiffUs << ","
                << d.sysDiffUs << ","
                << d.timestampUs << std::endl;
    }

    csvFile.close();
    std::cout << "CSV exported: " << path << " (" << allDiffs_.size() << " rows)" << std::endl;
}