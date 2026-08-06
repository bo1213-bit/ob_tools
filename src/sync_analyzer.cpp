// sync_analyzer.cpp
// 模块2: 时间戳对比分析 — 三类对比 + 统计 + CSV 导出

#include "sync_analyzer.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

std::tuple<std::vector<int64_t>, std::vector<int64_t>, std::vector<int64_t>, std::vector<int64_t>>
SyncAnalyzer::matchAndDiff(
    const std::vector<FrameStamp>& a,
    const std::vector<FrameStamp>& b,
    int64_t hwThresholdUs,
    bool useGlobalTimestamp)
{
    std::vector<int64_t> matchDiffs, sysDiffs, timestamps, hwDiffs;

    if (a.empty() || b.empty()) {
        return {matchDiffs, sysDiffs, timestamps, hwDiffs};
    }

    for (size_t mi = 0; mi < a.size(); mi++) {
        int64_t bestDist = INT64_MAX;
        size_t  bestIdx  = 0;

        for (size_t mj = 0; mj < b.size(); mj++) {
            int64_t dist;
            if (useGlobalTimestamp) {
                dist = std::abs(a[mi].globalTimestampUs - b[mj].globalTimestampUs);
            } else {
                dist = std::abs(a[mi].hwTimestampUs - b[mj].hwTimestampUs);
            }
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx  = mj;
            }
        }

        if (bestDist < hwThresholdUs) {
            if (useGlobalTimestamp) {
                matchDiffs.push_back(a[mi].globalTimestampUs - b[bestIdx].globalTimestampUs);
            } else {
                matchDiffs.push_back(a[mi].hwTimestampUs - b[bestIdx].hwTimestampUs);
            }
            hwDiffs.push_back(a[mi].hwTimestampUs - b[bestIdx].hwTimestampUs);
            sysDiffs.push_back(a[mi].sysTimestampUs - b[bestIdx].sysTimestampUs);
            timestamps.push_back(a[mi].sysTimestampUs);  // reference frame's system timestamp
        }
    }

    return {matchDiffs, sysDiffs, timestamps, hwDiffs};
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

    // 1. 同设备跨流: Depth vs Color
    for (int i = 0; i < deviceCount; i++) {
        if (frames[i][DEPTH_IDX].empty() || frames[i][COLOR_IDX].empty()) continue;

        auto [hwDiffs, sysDiffs, timestamps, hwDiffs2] = matchAndDiff(
            frames[i][DEPTH_IDX], frames[i][COLOR_IDX], cfg.hwThresholdUs);

        auto stats = computeStats(i, i, StreamType::DEPTH, true, hwDiffs, sysDiffs);
        crossStreamStats_.push_back(stats);

        for (size_t k = 0; k < hwDiffs.size(); k++) {
            // 计算 globalTimestampUs 差值
            int64_t globalDiff = frames[i][DEPTH_IDX][k].globalTimestampUs
                               - frames[i][COLOR_IDX][k].globalTimestampUs;
            allDiffs_.push_back({"cross_stream", i, i, "depth+color", hwDiffs[k], globalDiff, sysDiffs[k], timestamps[k]});
        }
    }

    // 2. 跨设备同流 Depth (使用 globalTimestampUs 匹配)
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][DEPTH_IDX].empty() || frames[j][DEPTH_IDX].empty()) continue;

            auto [timeDiffs, sysDiffs, timestamps, hwDiffs] = matchAndDiff(
                frames[i][DEPTH_IDX], frames[j][DEPTH_IDX], cfg.hwThresholdUs, true);

            auto stats = computeStats(i, j, StreamType::DEPTH, false, timeDiffs, sysDiffs);
            crossDeviceDepthStats_.push_back(stats);

            for (size_t k = 0; k < timeDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "depth", hwDiffs[k], timeDiffs[k], sysDiffs[k], timestamps[k]});
            }
        }
    }

    // 3. 跨设备同流 Color (使用 globalTimestampUs 匹配)
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][COLOR_IDX].empty() || frames[j][COLOR_IDX].empty()) continue;

            auto [timeDiffs, sysDiffs, timestamps, hwDiffs] = matchAndDiff(
                frames[i][COLOR_IDX], frames[j][COLOR_IDX], cfg.hwThresholdUs, true);

            auto stats = computeStats(i, j, StreamType::COLOR, false, timeDiffs, sysDiffs);
            crossDeviceColorStats_.push_back(stats);

            for (size_t k = 0; k < timeDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "color", hwDiffs[k], timeDiffs[k], sysDiffs[k], timestamps[k]});
            }
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

void SyncAnalyzer::printOneStats(const PairStats& s) const {
    std::cout << "  Pair count: " << s.pairCount << std::endl;
    std::cout << "  HW Timestamp Diff:" << std::endl;
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