// frame_stamp.h
// Shared data structures: StreamType enum + FrameStamp struct
// ROS2 package independent copy — no dependency on libobsensor

#pragma once

#include <cstdint>

enum class StreamType {
    DEPTH = 0,
    COLOR = 1
};

struct FrameStamp {
    int64_t    hwTimestampUs;   // Hardware timestamp (from header.stamp initially)
    int64_t    sysTimestampUs;  // Software/system timestamp (header.stamp)
    int64_t    frameNumber;     // Frame sequence number
    int        deviceIndex;     // Device index (0, 1, 2, ...)
    StreamType streamType;      // DEPTH or COLOR
};