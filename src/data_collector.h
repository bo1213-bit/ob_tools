// data_collector.h
// 模块1: 数据采集 — 枚举设备、配置硬同步、采集 Depth+Color 帧

#pragma once

#include "frame_stamp.h"
#include <libobsensor/ObSensor.hpp>
#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class DataCollector {
public:
    struct Config {
        int64_t  durationSec = 300;
        int64_t  width       = 1280;
        int64_t  height      = 800;
        int64_t  fps         = 30;
        bool     useDepth    = true;
        bool     useColor    = true;
        std::string outputDir;   // 非空则把采集到的 color 帧保存为 PNG + 写 timestamps.csv
    };

    // 执行完整采集流程: 枚举 → 配置同步 → 复位时钟 → 采集 → 停止
    void run(const Config& cfg);

    // 外部调用（如信号处理），通知采集提前停止
    void stop();

    // 将采集到的所有帧导出为原始时间戳 CSV
    // 格式: deviceIndex,streamType,hwTimestampUs,globalTimestampUs,sysTimestampUs
    void exportRawCSV(const std::string& path) const;

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

    // 把 color 帧保存为 PNG 并写入 timestamps.csv（outputDir_ 非空时由回调调用）
    void saveColorImage(const std::shared_ptr<ob::Frame>& colorFrame, int camIndex);

    // 成员变量
    std::shared_ptr<ob::Context>                           context_;
    std::vector<std::shared_ptr<ob::Device>>               devices_;
    std::vector<std::shared_ptr<ob::Pipeline>>             pipelines_;
    // 回调中直接写入 allFrames_[deviceIndex][streamType]，加锁保护
    std::vector<std::vector<std::shared_ptr<std::mutex>>>  mutexes_;       // [deviceIndex][streamType]
    // 最终结果: [deviceIndex][streamType][frameIndex]
    std::vector<std::vector<std::vector<FrameStamp>>>      allFrames_;
    // 采集运行标志，stop() 设为 false，collectFrames 中轮询检查
    std::atomic<bool>                                      running_{true};

    // ---- 图像保存 ----
    std::string             outputDir_;      // 保存目录，非空则启用图像输出
    std::ofstream           csvFile_;        // timestamps.csv
    std::mutex              csvMutex_;       // 保护 csvFile_ 与 globalSeq_
    std::vector<int>        savedCount_;     // [deviceIndex] 已保存的帧序号
    int                     globalSeq_ = 0;  // 全局帧序号 (作为 groupId)
};