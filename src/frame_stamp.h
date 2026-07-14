// frame_stamp.h
// 共享数据结构：StreamType 枚举 + FrameStamp 结构体
// 被 DataCollector 和 SyncAnalyzer 共同使用

#pragma once

#include <cstdint>

enum class StreamType {
    DEPTH = 0,
    COLOR = 1
};

struct FrameStamp {
    int64_t    hwTimestampUs;   // 硬件时间戳 (OB_FRAME_METADATA_TYPE_TIMESTAMP)
    int64_t    sysTimestampUs;  // 软件时间戳 (std::chrono::system_clock at callback)
    int64_t    frameNumber;     // 帧序号 (OB_FRAME_METADATA_TYPE_FRAME_NUMBER)
    int        deviceIndex;     // 设备索引 (0, 1, 2, ...)
    StreamType streamType;      // DEPTH 或 COLOR
};