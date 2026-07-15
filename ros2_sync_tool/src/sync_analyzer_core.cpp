// sync_analyzer_core.cpp
// Pure analysis logic: frame pairing, diff calculation, statistics, CSV export.

#include "sync_analyzer_core.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

// ---------------------------------------------------------------------------
// matchAndDiff — find best-matching frames by hardware timestamp
// ---------------------------------------------------------------------------
std::pair<std::vector<int64_t>, std::vector<int64_t>>
SyncAnalyzerCore::matchAndDiff(
    const std::vector<FrameStamp>& a,
    const std::vector<FrameStamp>& b,
    int64_t hwThresholdUs)
{
    std::vector<int64_t> hwDiffs, sysDiffs;

    if (a.empty() || b.empty()) {
        return {hwDiffs, sysDiffs};
    }

    for (size_t mi = 0; mi < a.size(); mi++) {
        int64_t bestDist = INT64_MAX;
        size_t  bestIdx  = 0;

        for (size_t mj = 0; mj < b.size(); mj++) {
            int64_t dist = std::abs(a[mi].hwTimestampUs - b[mj].hwTimestampUs);
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx  = mj;
            }
        }

        if (bestDist < hwThresholdUs) {
            hwDiffs.push_back(a[mi].hwTimestampUs - b[bestIdx].hwTimestampUs);
            sysDiffs.push_back(a[mi].sysTimestampUs - b[bestIdx].sysTimestampUs);
        }
    }

    return {hwDiffs, sysDiffs};
}

// ---------------------------------------------------------------------------
// computeStats — aggregate diff vectors into PairStats
// ---------------------------------------------------------------------------
SyncAnalyzerCore::PairStats
SyncAnalyzerCore::computeStats(
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

    auto calc = [](const std::vector<int64_t>& diffs,
                   int64_t& minVal, int64_t& maxVal,
                   double& mean, double& stddev) {
        std::vector<int64_t> sorted = diffs;
        std::sort(sorted.begin(), sorted.end());
        minVal = sorted.front();
        maxVal = sorted.back();

        double sum = 0.0;
        for (auto d : diffs) sum += static_cast<double>(d);
        mean = sum / static_cast<double>(diffs.size());

        double sqSum = 0.0;
        for (auto d : diffs) {
            double delta = static_cast<double>(d) - mean;
            sqSum += delta * delta;
        }
        stddev = std::sqrt(sqSum / static_cast<double>(diffs.size()));
    };

    calc(hwDiffs,  s.hwMinUs,  s.hwMaxUs,  s.hwMeanUs,  s.hwStddevUs);
    calc(sysDiffs, s.sysMinUs, s.sysMaxUs, s.sysMeanUs, s.sysStddevUs);

    return s;
}

// ---------------------------------------------------------------------------
// run — three-way comparison
// ---------------------------------------------------------------------------
void SyncAnalyzerCore::run(
    const std::vector<std::vector<std::vector<FrameStamp>>>& frames,
    const Config& cfg)
{
    int deviceCount = static_cast<int>(frames.size());
    const int DEPTH_IDX = static_cast<int>(StreamType::DEPTH);
    const int COLOR_IDX = static_cast<int>(StreamType::COLOR);

    // Clear previous results
    crossStreamStats_.clear();
    crossDeviceDepthStats_.clear();
    crossDeviceColorStats_.clear();
    allDiffs_.clear();

    // 1. Cross-stream: Depth vs Color (same device)
    for (int i = 0; i < deviceCount; i++) {
        if (frames[i][DEPTH_IDX].empty() || frames[i][COLOR_IDX].empty()) continue;

        auto [hwDiffs, sysDiffs] = matchAndDiff(
            frames[i][DEPTH_IDX], frames[i][COLOR_IDX], cfg.hwThresholdUs);

        auto stats = computeStats(i, i, StreamType::DEPTH, true, hwDiffs, sysDiffs);
        crossStreamStats_.push_back(stats);

        for (size_t k = 0; k < hwDiffs.size(); k++) {
            allDiffs_.push_back({"cross_stream", i, i, "depth+color", hwDiffs[k], sysDiffs[k]});
        }
    }

    // 2. Cross-device Depth
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][DEPTH_IDX].empty() || frames[j][DEPTH_IDX].empty()) continue;

            auto [hwDiffs, sysDiffs] = matchAndDiff(
                frames[i][DEPTH_IDX], frames[j][DEPTH_IDX], cfg.hwThresholdUs);

            auto stats = computeStats(i, j, StreamType::DEPTH, false, hwDiffs, sysDiffs);
            crossDeviceDepthStats_.push_back(stats);

            for (size_t k = 0; k < hwDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "depth", hwDiffs[k], sysDiffs[k]});
            }
        }
    }

    // 3. Cross-device Color
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][COLOR_IDX].empty() || frames[j][COLOR_IDX].empty()) continue;

            auto [hwDiffs, sysDiffs] = matchAndDiff(
                frames[i][COLOR_IDX], frames[j][COLOR_IDX], cfg.hwThresholdUs);

            auto stats = computeStats(i, j, StreamType::COLOR, false, hwDiffs, sysDiffs);
            crossDeviceColorStats_.push_back(stats);

            for (size_t k = 0; k < hwDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "color", hwDiffs[k], sysDiffs[k]});
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------
const std::vector<SyncAnalyzerCore::PairStats>& SyncAnalyzerCore::getCrossStreamStats() const {
    return crossStreamStats_;
}
const std::vector<SyncAnalyzerCore::PairStats>& SyncAnalyzerCore::getCrossDeviceDepthStats() const {
    return crossDeviceDepthStats_;
}
const std::vector<SyncAnalyzerCore::PairStats>& SyncAnalyzerCore::getCrossDeviceColorStats() const {
    return crossDeviceColorStats_;
}

// ---------------------------------------------------------------------------
// printOneStats — single PairStats output
// ---------------------------------------------------------------------------
void SyncAnalyzerCore::printOneStats(const PairStats& s) {
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

// ---------------------------------------------------------------------------
// printReport — full console report
// ---------------------------------------------------------------------------
void SyncAnalyzerCore::printReport() const {
    std::cout << "\n==============================================" << std::endl;
    std::cout << "  Timestamp Sync Analysis Report" << std::endl;
    std::cout << "==============================================" << std::endl;

    std::cout << "\n--- 1. Cross-Stream (Depth vs Color) ---" << std::endl;
    if (crossStreamStats_.empty()) {
        std::cout << "  (no data)" << std::endl;
    } else {
        for (const auto& s : crossStreamStats_) {
            std::cout << "Device " << s.deviceI << ":" << std::endl;
            printOneStats(s);
        }
    }

    std::cout << "\n--- 2. Cross-Device Depth ---" << std::endl;
    if (crossDeviceDepthStats_.empty()) {
        std::cout << "  (no data)" << std::endl;
    } else {
        for (const auto& s : crossDeviceDepthStats_) {
            std::cout << "Device " << s.deviceI << " vs Device " << s.deviceJ << ":" << std::endl;
            printOneStats(s);
        }
    }

    std::cout << "\n--- 3. Cross-Device Color ---" << std::endl;
    if (crossDeviceColorStats_.empty()) {
        std::cout << "  (no data)" << std::endl;
    } else {
        for (const auto& s : crossDeviceColorStats_) {
            std::cout << "Device " << s.deviceI << " vs Device " << s.deviceJ << ":" << std::endl;
            printOneStats(s);
        }
    }

    std::cout << "==============================================" << std::endl;
}

// ---------------------------------------------------------------------------
// exportCSV — write raw diffs to CSV
// ---------------------------------------------------------------------------
void SyncAnalyzerCore::exportCSV(const std::string& path) const {
    std::ofstream csvFile(path);
    if (!csvFile.is_open()) {
        std::cerr << "Cannot open CSV: " << path << std::endl;
        return;
    }

    csvFile << "comparison_type,device_i,device_j,stream,hw_diff_us,sys_diff_us" << std::endl;

    for (const auto& d : allDiffs_) {
        csvFile << d.comparisonType << ","
                << d.deviceI << ","
                << d.deviceJ << ","
                << d.streamLabel << ","
                << d.hwDiffUs << ","
                << d.sysDiffUs << std::endl;
    }

    csvFile.close();
    std::cout << "CSV exported: " << path << " (" << allDiffs_.size() << " rows)" << std::endl;
}