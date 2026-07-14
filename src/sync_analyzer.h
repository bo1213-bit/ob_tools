// sync_analyzer.h
// 模块2: 时间戳对比分析 — 三类对比 + 统计 + CSV 导出

#pragma once

#include "frame_stamp.h"
#include <libobsensor/ObSensor.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class SyncAnalyzer {
public:
    struct Config {
        int64_t hwThresholdUs = 500;  // 硬件时间戳配对阈值 (us)
    };

    struct PairStats {
        int        deviceI, deviceJ;   // 设备索引
        StreamType streamType;          // DEPTH 或 COLOR (跨设备时有效)
        bool       isCrossStream;       // true=同设备跨流, false=跨设备同流
        int        pairCount;           // 成功配对帧数

        // 硬件时间戳差统计 (us)
        int64_t    hwMinUs, hwMaxUs;
        double     hwMeanUs, hwStddevUs;

        // 全局(软件)时间戳差统计 (us)
        int64_t    sysMinUs, sysMaxUs;
        double     sysMeanUs, sysStddevUs;
    };

    // 执行分析: 三类对比 → 统计
    void run(
        const std::vector<std::vector<std::vector<FrameStamp>>>& frames,
        const std::vector<std::shared_ptr<ob::Device>>& devices,
        const Config& cfg
    );

    // 获取三类对比结果
    const std::vector<PairStats>& getCrossStreamStats() const;       // 同设备 Depth vs Color
    const std::vector<PairStats>& getCrossDeviceDepthStats() const;  // 跨设备 Depth vs Depth
    const std::vector<PairStats>& getCrossDeviceColorStats() const;  // 跨设备 Color vs Color

    // 输出控制台报告
    void printReport() const;

    // 导出 CSV (包含所有对比的原始 diff)
    void exportCSV(const std::string& path) const;

private:
    // 通用配对: 两组 FrameStamp → 硬件时间戳差 + 软件时间戳差
    static std::pair<std::vector<int64_t>, std::vector<int64_t>>
    matchAndDiff(
        const std::vector<FrameStamp>& a,
        const std::vector<FrameStamp>& b,
        int64_t hwThresholdUs
    );

    // 根据 diff 向量计算 PairStats
    static PairStats computeStats(
        int devI, int devJ,
        StreamType st, bool isCrossStream,
        const std::vector<int64_t>& hwDiffs,
        const std::vector<int64_t>& sysDiffs
    );

    // 打印单组统计
    void printOneStats(const PairStats& s) const;

    // 成员变量
    std::vector<PairStats> crossStreamStats_;       // 同设备跨流
    std::vector<PairStats> crossDeviceDepthStats_;  // 跨设备 Depth
    std::vector<PairStats> crossDeviceColorStats_;  // 跨设备 Color

    // 原始 diff 数据 (用于 CSV 导出)
    struct DiffRecord {
        std::string comparisonType;  // "cross_stream" | "cross_device"
        int         deviceI, deviceJ;
        std::string streamLabel;     // "depth+color" | "depth" | "color"
        int64_t     hwDiffUs;
        int64_t     sysDiffUs;
    };
    std::vector<DiffRecord> allDiffs_;
};