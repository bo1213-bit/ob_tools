// sync_analyzer.h
// 模块2: 时间戳对比分析 — 三类对比 + 统计 + CSV 导出

#pragma once

#include "frame_stamp.h"
#include <libobsensor/ObSensor.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

class SyncAnalyzer {
public:
    struct Config {
        int64_t globalThresholdUs = 5000;  // 后匹配"异常"判定阈值 (us): 组内 global 极差 >= 此值记为异常 (对应官方 --threshold)
        double   fps           = 30.0;  // 帧率: 配对容差 matchTolUs = 1e6/fps/2 (与官方 half_gap_us 一致)
    };

    struct PairStats {
        int        deviceI, deviceJ;   // 设备索引
        StreamType streamType;          // DEPTH 或 COLOR (跨设备时有效)
        bool       isCrossStream;       // true=同设备跨流, false=跨设备同流
        int        pairCount;           // 成功配对帧数

        // 全局(跨设备时钟域)时间戳差统计 (us) —— 同步精度的正确度量
        // (globalTimeStampUs 是换算到主机时钟域的时间戳; 官方 analyze_sync.py 默认 time base = global。
        //  匹配与被度量都用 global, 前提是采集端按官方顺序完成时钟同步, 见 resetTimestampAndSyncClock)
        int64_t    globalMinUs, globalMaxUs;
        double     globalMeanUs, globalStddevUs;

        // 设备端硬件时间戳差统计 (us) —— 仅参考 (timeStampUs, 设备本地时钟)
        int64_t    hwMinUs, hwMaxUs;
        double     hwMeanUs, hwStddevUs;

        // 主机端系统时间戳差统计 (us)
        int64_t    sysMinUs, sysMaxUs;
        double     sysMeanUs, sysStddevUs;
    };

    struct MultiDeviceStats {
        StreamType streamType;          // DEPTH 或 COLOR
        int        deviceCount;         // 参与匹配的设备数
        int        matchCount;          // 成功匹配组数
        int        abnormalCount;       // 组内极差 >= abnormalThresholdUs 的组数 (异常)
        int64_t    abnormalThresholdUs; // 异常判定阈值 (us)
        int64_t    globalMinUs, globalMaxUs;  // 每组 全局(global)时间戳极差 的最小/最大值 —— 同步精度的正确度量
        double     globalMeanUs, globalStddevUs;
    };

    // 执行分析: 三类对比 + 多设备匹配 → 统计
    void run(
        const std::vector<std::vector<std::vector<FrameStamp>>>& frames,
        const std::vector<std::shared_ptr<ob::Device>>& devices,
        const Config& cfg
    );

    // 获取三类对比结果
    const std::vector<PairStats>& getCrossStreamStats() const;       // 同设备 Depth vs Color
    const std::vector<PairStats>& getCrossDeviceDepthStats() const;  // 跨设备 Depth vs Depth
    const std::vector<PairStats>& getCrossDeviceColorStats() const;  // 跨设备 Color vs Color
    const MultiDeviceStats& getMultiDeviceDepthStats() const;        // 多设备同步 Depth
    const MultiDeviceStats& getMultiDeviceColorStats() const;        // 多设备同步 Color

    // 输出控制台报告
    void printReport() const;

    // 导出 CSV (包含所有对比的原始 diff)
    void exportCSV(const std::string& path) const;

private:
    // 通用配对: 先按 globalTimestampUs 排序，再按官方前向游标规则一对一匹配。
    // 已选中的 b 帧会被消费，不会复用于后续 a 帧。
    // 返回 {hwDiffs, globalDiffs, sysDiffs, timestamps, aIndices, bIndices}:
    //   globalDiffs: 全局时间戳差 globalTimeStampUs —— 同步精度的正确度量(匹配与被度量都用 global)
    //   hwDiffs:     设备端时间戳差 timeStampUs —— 仅参考(设备本地时钟)
    //   sysDiffs:    主机端时间戳差 systemTimeStampUs
    //   timestamps:  参考帧主机时间戳; aIndices/bIndices: 匹配对在两组中的索引
    static std::tuple<std::vector<int64_t>, std::vector<int64_t>, std::vector<int64_t>, std::vector<int64_t>, std::vector<size_t>, std::vector<size_t>>
    matchAndDiff(
        const std::vector<FrameStamp>& a,
        const std::vector<FrameStamp>& b,
        int64_t matchTolUs
    );

    // 根据 diff 向量计算 PairStats
    static PairStats computeStats(
        int devI, int devJ,
        StreamType st, bool isCrossStream,
        const std::vector<int64_t>& hwDiffs,
        const std::vector<int64_t>& globalDiffs,
        const std::vector<int64_t>& sysDiffs
    );

    // 多设备匹配原始 diff: global = max(global)-min(global) (同步精度), hw = max(hw)-min(hw) (参考)
    struct MultiDeviceDiffs {
        std::vector<int64_t> hw;
        std::vector<int64_t> global;
    };

    // 多设备最近邻匹配: 所有设备同一流类型 → 每组 max(global)-min(global) 作为同步精度
    // 以指定基准设备(如 335Lg)为准，每帧在其余设备中找匹配容差内最近的帧(按全局 global 时间戳);
    // refDevOverride<0 时退回帧数最少设备
    MultiDeviceDiffs multiDeviceMatch(
        const std::vector<std::vector<FrameStamp>>& allDevFrames,
        int64_t matchTolUs,
        int refDevOverride = -1
    );

    // 打印单组统计
    void printOneStats(const PairStats& s) const;

    // 成员变量
    std::vector<PairStats> crossStreamStats_;       // 同设备跨流
    std::vector<PairStats> crossDeviceDepthStats_;  // 跨设备 Depth
    std::vector<PairStats> crossDeviceColorStats_;  // 跨设备 Color
    MultiDeviceStats       multiDeviceDepthStats_;  // 多设备同步 Depth
    MultiDeviceStats       multiDeviceColorStats_;  // 多设备同步 Color

    // 多设备匹配原始 diff (用于 CSV): global 同步精度 / hw 参考
    MultiDeviceDiffs       multiDeviceDepthDiffs_;
    MultiDeviceDiffs       multiDeviceColorDiffs_;

    std::string            multiRefName_;        // 多设备匹配的基准设备名 (如 335Lg)

    // 原始 diff 数据 (用于 CSV 导出)
    struct DiffRecord {
        std::string comparisonType;  // "cross_stream" | "cross_device"
        int         deviceI, deviceJ;
        std::string streamLabel;     // "depth+color" | "depth" | "color"
        int64_t     hwDiffUs;        // 设备端时间戳差 (timeStampUs)
        int64_t     globalDiffUs;    // 全局时间戳差 (globalTimeStampUs)
        int64_t     sysDiffUs;       // 主机端时间戳差 (systemTimeStampUs)
        int64_t     timestampUs;     // 参考帧的设备端时间戳
    };
    std::vector<DiffRecord> allDiffs_;
};