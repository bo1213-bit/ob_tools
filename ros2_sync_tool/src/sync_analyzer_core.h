// sync_analyzer_core.h
// Pure analysis logic: frame pairing, diff calculation, statistics, CSV export.
// No dependency on libobsensor or ROS2 — only uses FrameStamp and standard library.

#pragma once

#include "frame_stamp.h"

#include <cstdint>
#include <string>
#include <vector>

class SyncAnalyzerCore {
public:
    struct Config {
        int64_t hwThresholdUs = 500;  // Matching threshold for timestamps (us)
    };

    struct PairStats {
        int        deviceI, deviceJ;   // Device indices
        StreamType streamType;         // DEPTH or COLOR (for cross-device comparisons)
        bool       isCrossStream;      // true=same-device cross-stream, false=cross-device same-stream
        int        pairCount;          // Number of successfully paired frames

        // Hardware timestamp diff statistics (us)
        int64_t    hwMinUs, hwMaxUs;
        double     hwMeanUs, hwStddevUs;

        // Global (software) timestamp diff statistics (us)
        int64_t    sysMinUs, sysMaxUs;
        double     sysMeanUs, sysStddevUs;
    };

    struct DiffRecord {
        std::string comparisonType;  // "cross_stream" | "cross_device"
        int         deviceI, deviceJ;
        std::string streamLabel;     // "depth+color" | "depth" | "color"
        int64_t     hwDiffUs;
        int64_t     sysDiffUs;
    };

    // Run analysis on collected frames.
    // frames[deviceIndex][streamTypeIndex][frameIndex]
    void run(const std::vector<std::vector<std::vector<FrameStamp>>>& frames,
             const Config& cfg);

    // Access results
    const std::vector<PairStats>& getCrossStreamStats() const;
    const std::vector<PairStats>& getCrossDeviceDepthStats() const;
    const std::vector<PairStats>& getCrossDeviceColorStats() const;

    // Output
    void printReport() const;
    void exportCSV(const std::string& path) const;

private:
    // Match two FrameStamp sequences by hardware timestamp within threshold.
    // Returns (hwDiffs, sysDiffs) vectors.
    static std::pair<std::vector<int64_t>, std::vector<int64_t>>
    matchAndDiff(const std::vector<FrameStamp>& a,
                 const std::vector<FrameStamp>& b,
                 int64_t hwThresholdUs);

    // Compute statistics from diff vectors.
    static PairStats computeStats(int devI, int devJ,
                                  StreamType st, bool isCrossStream,
                                  const std::vector<int64_t>& hwDiffs,
                                  const std::vector<int64_t>& sysDiffs);

    // Print a single PairStats to stdout.
    static void printOneStats(const PairStats& s);

    // --- Members ---
    std::vector<PairStats> crossStreamStats_;
    std::vector<PairStats> crossDeviceDepthStats_;
    std::vector<PairStats> crossDeviceColorStats_;
    std::vector<DiffRecord>  allDiffs_;
};