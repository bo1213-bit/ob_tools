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
    int64_t    hwTimestampUs;   // 硬件时间戳 (frame->timeStampUs(), 即设备端时间)
    int64_t    globalTimestampUs; // 全局时间戳 (frame->globalTimeStampUs(), 即跨设备同步时钟)
    int64_t    sysTimestampUs;  // 系统时间戳 (frame->systemTimeStampUs(), 即主机端时间)
    int64_t    frameNumber;     // 帧序号
    int        deviceIndex;     // 设备索引 (0, 1, 2, ...)
    StreamType streamType;      // DEPTH 或 COLOR
};