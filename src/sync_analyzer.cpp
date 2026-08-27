// sync_analyzer.cpp
// 模块2: 时间戳对比分析 — 三类对比 + 统计 + CSV 导出

#include "sync_analyzer.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

std::tuple<std::vector<int64_t>, std::vector<int64_t>, std::vector<int64_t>, std::vector<int64_t>, std::vector<size_t>, std::vector<size_t>>
SyncAnalyzer::matchAndDiff(
    const std::vector<FrameStamp>& a,
    const std::vector<FrameStamp>& b,
    int64_t hwThresholdUs,
    bool useGlobalTimestamp)
{
    std::vector<int64_t> matchDiffs, sysDiffs, timestamps, hwDiffs;
    std::vector<size_t> aIndices, bIndices;

    if (a.empty() || b.empty()) {
        return {matchDiffs, sysDiffs, timestamps, hwDiffs, aIndices, bIndices};
    }

    // Pre-compute b timestamps for fast lookup
    std::vector<int64_t> bTimestamps(b.size());
    for (size_t j = 0; j < b.size(); j++) {
        bTimestamps[j] = useGlobalTimestamp ? b[j].globalTimestampUs : b[j].hwTimestampUs;
    }

    for (size_t mi = 0; mi < a.size(); mi++) {
        int64_t aTs = useGlobalTimestamp ? a[mi].globalTimestampUs : a[mi].hwTimestampUs;
        int64_t bestDist = INT64_MAX;
        size_t  bestIdx  = 0;

        for (size_t mj = 0; mj < b.size(); mj++) {
            int64_t dist = std::abs(aTs - bTimestamps[mj]);
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx  = mj;
            }
        }

        if (bestDist < hwThresholdUs) {
            matchDiffs.push_back(aTs - bTimestamps[bestIdx]);
            hwDiffs.push_back(a[mi].hwTimestampUs - b[bestIdx].hwTimestampUs);
            sysDiffs.push_back(a[mi].sysTimestampUs - b[bestIdx].sysTimestampUs);
            timestamps.push_back(a[mi].sysTimestampUs);
            aIndices.push_back(mi);
            bIndices.push_back(bestIdx);
        }
    }

    return {matchDiffs, sysDiffs, timestamps, hwDiffs, aIndices, bIndices};
}

SyncAnalyzer::MultiDeviceDiffs SyncAnalyzer::multiDeviceMatch(
    const std::vector<std::vector<FrameStamp>>& allDevFrames,
    int64_t hwThresholdUs)
{
    MultiDeviceDiffs result;

    // Need at least 2 devices with frames
    std::vector<int> validDevs;
    for (int i = 0; i < static_cast<int>(allDevFrames.size()); i++) {
        if (!allDevFrames[i].empty()) validDevs.push_back(i);
    }
    if (validDevs.size() < 2) return result;

    // Pick reference device = fewest frames
    int refDev = validDevs[0];
    for (int d : validDevs) {
        if (allDevFrames[d].size() < allDevFrames[refDev].size()) {
            refDev = d;
        }
    }
    std::vector<int> otherDevs;
    for (int d : validDevs) {
        if (d != refDev) otherDevs.push_back(d);
    }

    // Pre-extract timestamps: globalTimestampUs for matching, hwTimestampUs for reference diff
    std::vector<std::vector<int64_t>> globalTs(allDevFrames.size());
    std::vector<std::vector<int64_t>> hwTs(allDevFrames.size());
    for (int d : validDevs) {
        globalTs[d].reserve(allDevFrames[d].size());
        hwTs[d].reserve(allDevFrames[d].size());
        for (const auto& f : allDevFrames[d]) {
            globalTs[d].push_back(f.globalTimestampUs);
            hwTs[d].push_back(f.hwTimestampUs);
        }
    }

    const auto& refGlobal = globalTs[refDev];
    const auto& refHw = hwTs[refDev];
    for (size_t ri = 0; ri < refGlobal.size(); ri++) {
        // Match on globalTimestampUs (host-synced, consistent across devices)
        int64_t refGlobalTime = refGlobal[ri];
        std::vector<int64_t> groupHw = {refHw[ri]};
        std::vector<int64_t> groupGlobal = {refGlobal[ri]};
        bool allMatched = true;

        for (int d : otherDevs) {
            const auto& gArr = globalTs[d];
            const auto& hArr = hwTs[d];

            int64_t bestDist = INT64_MAX;
            size_t bestIdx = 0;
            for (size_t j = 0; j < gArr.size(); j++) {
                int64_t dist = std::abs(gArr[j] - refGlobalTime);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = j;
                }
            }

            if (bestDist >= hwThresholdUs) {
                allMatched = false;
                break;
            }
            // global 用于同步精度, hw 保留作参考
            groupHw.push_back(hArr[bestIdx]);
            groupGlobal.push_back(gArr[bestIdx]);
        }

        if (allMatched) {
            auto [hminIt, hmaxIt] = std::minmax_element(groupHw.begin(), groupHw.end());
            result.hw.push_back(*hmaxIt - *hminIt);
            auto [gminIt, gmaxIt] = std::minmax_element(groupGlobal.begin(), groupGlobal.end());
            result.global.push_back(*gmaxIt - *gminIt);
        }
    }

    return result;
}

SyncAnalyzer::PairStats
SyncAnalyzer::computeStats(
    int devI, int devJ,
    StreamType st, bool isCrossStream,
    const std::vector<int64_t>& hwDiffs,
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
    calc(sysDiffs, s.sysMinUs, s.sysMaxUs, s.sysMeanUs, s.sysStddevUs);

    return s;
}

void SyncAnalyzer::run(
    const std::vector<std::vector<std::vector<FrameStamp>>>& frames,
    const std::vector<std::shared_ptr<ob::Device>>& /*devices*/,
    const Config& cfg)
{
    int deviceCount = static_cast<int>(frames.size());
    const int DEPTH_IDX = static_cast<int>(StreamType::DEPTH);
    const int COLOR_IDX = static_cast<int>(StreamType::COLOR);

    // 1. 同设备跨流: Depth vs Color (用 globalTimestampUs 匹配, 差值报告 global)
    for (int i = 0; i < deviceCount; i++) {
        if (frames[i][DEPTH_IDX].empty() || frames[i][COLOR_IDX].empty()) continue;

        auto [globalDiffs, sysDiffs, timestamps, hwDiffs, aIdx, bIdx] = matchAndDiff(
            frames[i][DEPTH_IDX], frames[i][COLOR_IDX], cfg.hwThresholdUs, true);

        auto stats = computeStats(i, i, StreamType::DEPTH, true, globalDiffs, sysDiffs);
        crossStreamStats_.push_back(stats);

        for (size_t k = 0; k < globalDiffs.size(); k++) {
            allDiffs_.push_back({"cross_stream", i, i, "depth+color", hwDiffs[k], globalDiffs[k], sysDiffs[k], timestamps[k]});
        }
    }

    // 2. 跨设备同流 Depth (使用 globalTimestampUs 匹配, 差值报告 global)
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][DEPTH_IDX].empty() || frames[j][DEPTH_IDX].empty()) continue;

            auto [globalDiffs, sysDiffs, timestamps, hwDiffs, aIdx, bIdx] = matchAndDiff(
                frames[i][DEPTH_IDX], frames[j][DEPTH_IDX], cfg.hwThresholdUs, true);

            auto stats = computeStats(i, j, StreamType::DEPTH, false, globalDiffs, sysDiffs);
            crossDeviceDepthStats_.push_back(stats);

            for (size_t k = 0; k < globalDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "depth", hwDiffs[k], globalDiffs[k], sysDiffs[k], timestamps[k]});
            }
        }
    }

    // 3. 跨设备同流 Color (使用 globalTimestampUs 匹配, 差值报告 global)
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][COLOR_IDX].empty() || frames[j][COLOR_IDX].empty()) continue;

            auto [globalDiffs, sysDiffs, timestamps, hwDiffs, aIdx, bIdx] = matchAndDiff(
                frames[i][COLOR_IDX], frames[j][COLOR_IDX], cfg.hwThresholdUs, true);

            auto stats = computeStats(i, j, StreamType::COLOR, false, globalDiffs, sysDiffs);
            crossDeviceColorStats_.push_back(stats);

            for (size_t k = 0; k < globalDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "color", hwDiffs[k], globalDiffs[k], sysDiffs[k], timestamps[k]});
            }
        }
    }

    // 4. 多设备同步精度: 所有设备同一流类型, 匹配后 max(global)-min(global)
    {
        // Depth
        std::vector<std::vector<FrameStamp>> depthFrames(deviceCount);
        for (int i = 0; i < deviceCount; i++) {
            depthFrames[i] = frames[i][DEPTH_IDX];
        }
        multiDeviceDepthDiffs_ = multiDeviceMatch(depthFrames, cfg.hwThresholdUs);

        auto calcMdStats = [](const std::vector<int64_t>& diffs, StreamType st, int devCount) -> MultiDeviceStats {
            MultiDeviceStats s;
            s.streamType = st;
            s.deviceCount = devCount;
            s.matchCount = static_cast<int>(diffs.size());
            if (diffs.empty()) return s;
            auto [minIt, maxIt] = std::minmax_element(diffs.begin(), diffs.end());
            s.hwMinUs = *minIt;
            s.hwMaxUs = *maxIt;
            double sum = 0.0;
            for (auto d : diffs) sum += static_cast<double>(d);
            s.hwMeanUs = sum / diffs.size();
            double sqSum = 0.0;
            for (auto d : diffs) {
                double delta = static_cast<double>(d) - s.hwMeanUs;
                sqSum += delta * delta;
            }
            s.hwStddevUs = std::sqrt(sqSum / diffs.size());
            return s;
        };
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
        multiDeviceColorDiffs_ = multiDeviceMatch(colorFrames, cfg.hwThresholdUs);
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
    std::cout << "  Global Timestamp Diff:" << std::endl;
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
    {
        auto printMd = [](const std::string& label, const MultiDeviceStats& s) {
            std::cout << "  " << label << " (" << s.deviceCount << " devices):" << std::endl;
            std::cout << "    Match groups: " << s.matchCount << std::endl;
            if (s.matchCount > 0) {
                std::cout << "    max(global)-min(global): Min=" << s.hwMinUs << "us  Max=" << s.hwMaxUs << "us"
                          << "  Mean=" << std::fixed << std::setprecision(1) << s.hwMeanUs
                          << "us  Stddev=" << s.hwStddevUs << "us" << std::endl;
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