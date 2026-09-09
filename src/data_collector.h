// data_collector.h
// 模块1: 数据采集 — 枚举设备、配置硬同步、采集 Depth+Color 帧

#pragma once

#include "frame_stamp.h"
#include <libobsensor/ObSensor.hpp>
#include <opencv2/core.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
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
        int64_t  triggerHz   = 0;    // >0 时程序自动写 /sys/kernel/debug/gpio_trigger/framerate 控制外部触发
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

    // 后台写盘线程主体: 从 imageQueue_ 取图, 执行 imwrite + 写 timestamps.csv
    // (慢 I/O 移出回调线程, 避免阻塞 SDK 收帧导致丢帧)
    void writerLoop();

    struct PendingImage {
        std::string fname;
        cv::Mat     mat;
        int         groupId;
        int         deviceIndex;
        std::string deviceSN;
        uint64_t    deviceTimestampUs;
    };

    // ---- 采集诊断 ----
    // 只统计正式采集窗口的回调与帧序列，用于区分设备/SDK 出帧异常和主机回调积压。
    // 不参与匹配逻辑，也不改变 raw CSV 的既有格式。
    struct FrameEndpoint {
        bool       valid = false;
        FrameStamp stamp{};
        int64_t    metadataFrameNumber = -1;
        int64_t    callbackOffsetUs = 0;
    };

    struct StreamDiagnostics {
        uint64_t acceptedFrames = 0;
        uint64_t indexGapEvents = 0;
        uint64_t missingIndexFrames = 0;
        uint64_t indexRegressions = 0;
        uint64_t metadataGapEvents = 0;
        uint64_t missingMetadataFrames = 0;
        uint64_t metadataRegressions = 0;
        uint64_t hwTimestampRegressions = 0;
        uint64_t globalTimestampRegressions = 0;
        uint64_t systemTimestampRegressions = 0;
        uint64_t hwCadenceGaps = 0;
        uint64_t globalCadenceGaps = 0;
        uint64_t systemCadenceGaps = 0;
        uint64_t callbackCadenceGaps = 0;
        bool       hasPrevious = false;
        FrameStamp previous{};
        int64_t    previousMetadataFrameNumber = -1;
        int64_t    previousCallbackUs = 0;
        FrameEndpoint first;
        FrameEndpoint last;
        bool       profileObserved = false;
        uint32_t   observedWidth = 0;
        uint32_t   observedHeight = 0;
        int        observedFormat = 0;
    };

    struct DeviceDiagnostics {
        uint64_t closedGateCallbacks = 0;
        uint64_t openGateCallbacks = 0;
        uint64_t completeFrameSets = 0;
        uint64_t depthOnlyFrameSets = 0;
        uint64_t colorOnlyFrameSets = 0;
        uint64_t emptyFrameSets = 0;
        uint64_t imageSaveSamples = 0;
        int64_t  imageSaveTotalUs = 0;
        int64_t  imageSaveMaxUs = 0;
        StreamDiagnostics streams[2];
    };

    void recordFrameDiagnostics(int camIndex, int streamIndex, const FrameStamp& stamp,
                                int64_t metadataFrameNumber, int64_t callbackUs,
                                int64_t expectedPeriodUs);
    void printDiagnosticsSummary(const Config& cfg, int64_t expectedPeriodUs) const;

    // 成员变量
    std::shared_ptr<ob::Context>                           context_;
    std::vector<std::shared_ptr<ob::Device>>               devices_;
    std::vector<std::shared_ptr<ob::Pipeline>>             pipelines_;
    // 回调中直接写入 allFrames_[deviceIndex][streamType]，加锁保护
    std::vector<std::vector<std::shared_ptr<std::mutex>>>  mutexes_;       // [deviceIndex][streamType]
    // 最终结果: [deviceIndex][streamType][frameIndex]
    std::vector<std::vector<std::vector<FrameStamp>>>      allFrames_;
    std::vector<DeviceDiagnostics>                         diagnostics_;
    std::vector<std::shared_ptr<std::mutex>>                diagnosticMutexes_;
    // 正式窗口开启前写入 steady_clock 微秒基准；0 表示当前回调不计入正式窗口诊断。
    std::atomic<int64_t>                                   recordingArmSteadyUs_{0};
    // 采集运行标志，stop() 设为 false，collectFrames 中轮询检查
    std::atomic<bool>                                      running_{true};

    // ---- 图像保存 ----
    std::string             outputDir_;      // 保存目录，非空则启用图像输出
    std::ofstream           csvFile_;        // timestamps.csv
    std::mutex              csvMutex_;       // 保护 csvFile_ 与 globalSeq_
    std::vector<int>        savedCount_;     // [deviceIndex] 已保存的帧序号
    int                     globalSeq_ = 0;  // 全局帧序号 (作为 groupId)
    std::atomic<bool>       savingEnabled_{true};    // false 时 color 帧不入图像写盘队列
    // 正式采集窗口 gate: pipeline 启动期间和收尾冻结期间均为 false。
    // 自动触发时会短暂开启它只作 color 预热计数，随后清空这些预热帧；
    // 正式窗口外的回调不会向最终 allFrames_ 或 timestamps.csv 写入帧。
    std::atomic<bool>       recordingEnabled_{true};

    // ---- 后台写盘(慢 I/O 移出回调线程) ----
    std::queue<PendingImage> imageQueue_;         // 待落盘图片队列
    std::mutex               imageQueueMutex_;   // 保护 imageQueue_
    std::condition_variable  imageQueueCv_;      // 唤醒后台写盘线程
    std::thread              writerThread_;      // 后台写盘线程
    std::atomic<bool>        writerRunning_{false}; // 后台线程运行标志
};