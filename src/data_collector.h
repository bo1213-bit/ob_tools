// data_collector.h
// 模块1: 数据采集 — 枚举设备、配置硬同步、采集 Depth+Color 帧

#pragma once

#include "frame_stamp.h"
#include <libobsensor/ObSensor.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

class DataCollector {
public:
    struct Config {
        int  durationSec = 30;
        int  width       = 848;
        int  height      = 480;
        int  fps         = 30;
        bool useDepth    = true;
        bool useColor    = true;
    };

    // 执行完整采集流程: 枚举 → 配置同步 → 复位时钟 → 采集 → 停止
    void run(const Config& cfg);

    // 外部调用（如信号处理），通知采集提前停止
    void stop();

    // 返回采集到的帧数据
    // 三维数组: [deviceIndex][streamType][frameIndex]
    // streamType: 0 = DEPTH, 1 = COLOR
    const std::vector<std::vector<std::vector<FrameStamp>>>& getFrames() const;

    // 返回设备列表 (用于获取 SN 等信息)
    const std::vector<std::shared_ptr<ob::Device>>& getDevices() const;

private:
    void enumerateDevices();
    void configureSyncMode();
    void resetTimestampAndSyncClock();
    void collectFrames(const Config& cfg);

    // 成员变量
    std::shared_ptr<ob::Context>                           context_;
    std::vector<std::shared_ptr<ob::Device>>               devices_;
    std::vector<std::shared_ptr<ob::Pipeline>>             pipelines_;
    // 回调中直接写入 allFrames_[deviceIndex][streamType]，加锁保护
    std::vector<std::vector<std::mutex>>                   mutexes_;       // [deviceIndex][streamType]
    // 最终结果: [deviceIndex][streamType][frameIndex]
    std::vector<std::vector<std::vector<FrameStamp>>>      allFrames_;
    // 采集运行标志，stop() 设为 false，collectFrames 中轮询检查
    std::atomic<bool>                                      running_{true};
};