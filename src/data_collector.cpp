// data_collector.cpp
// 模块1: 数据采集

#include "data_collector.h"

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <exception>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <opencv2/opencv.hpp>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#define MKDIR(path) mkdir(path, 0755)
#endif

// 写外部触发频率到 debugfs 节点: /sys/kernel/debug/gpio_trigger/framerate
// 值单位 Hz; 0 表示关闭触发。需要 root 权限。返回是否写入成功。
static bool writeTriggerFramerate(int hz) {
    const char* path = "/sys/kernel/debug/gpio_trigger/framerate";
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "[WARN] cannot open trigger node " << path
                  << " (need root? run with sudo)" << std::endl;
        return false;
    }
    f << hz << std::endl;
    f.close();
    std::cout << "[TRIGGER] framerate set to " << hz << " Hz" << std::endl;
    return true;
}

void DataCollector::run(const Config& cfg) {
    running_ = true;
    std::cout << "=== DataCollector: Start ===" << std::endl;
    std::cout << "Config: " << cfg.width << "x" << cfg.height
              << " @ " << cfg.fps << "fps"
              << "  duration=" << cfg.durationSec << "s"
              << "  depth=" << (cfg.useDepth ? "on" : "off")
              << "  color=" << (cfg.useColor ? "on" : "off")
              << "  trigger=" << cfg.triggerHz << "Hz" << std::endl;

    enumerateDevices();
    configureSyncMode();
    resetTimestampAndSyncClock();
    collectFrames(cfg);

    std::cout << "=== DataCollector: Done ===" << std::endl;
}

void DataCollector::stop() {
    running_ = false;
}

const std::vector<std::vector<std::vector<FrameStamp>>>& DataCollector::getFrames() const {
    return allFrames_;
}

const std::vector<std::shared_ptr<ob::Device>>& DataCollector::getDevices() const {
    return devices_;
}

void DataCollector::enumerateDevices() {
    context_ = std::make_shared<ob::Context>();
    auto devList = context_->queryDeviceList();
    int devCount = devList->deviceCount();
    std::cout << "Found " << devCount << " device(s)" << std::endl;

    for (int i = 0; i < devCount; i++) {
        auto dev  = devList->getDevice(i);
        auto info = dev->getDeviceInfo();
        std::cout << "Device " << i << ": "
                  << "SN=" << info->serialNumber()
                  << "  Name=" << info->getName()
                  << "  PID=" << info->getPid()
                  << "  VID=" << info->getVid()
                  << "  FW=" << info->firmwareVersion()
                  << "  HW=" << info->hardwareVersion()
                  << std::endl;
        auto syncBitmap = dev->getSupportedMultiDeviceSyncModeBitmap();
        std::cout << "  Supported sync modes: 0x" << std::hex << syncBitmap << std::dec << std::endl;

        // 检查是否支持全局时间戳(global timestamp)——用于跨设备时钟对齐
        bool globalTsSupported = dev->isGlobalTimestampSupported();
        std::cout << "  Global timestamp supported: " << (globalTsSupported ? "YES" : "NO") << std::endl;

        devices_.push_back(dev);
    }

    if (devCount < 2) {
        throw std::runtime_error("Need at least 2 devices for sync check!");
    }
    std::cout << "\nDevices: " << devices_.size()
              << "  Pairs to check: " << (devices_.size() * (devices_.size() - 1) / 2)
              << std::endl;
}

void DataCollector::configureSyncMode() {
    for (size_t i = 0; i < devices_.size(); i++) {
        auto info = devices_[i]->getDeviceInfo();
        std::string connType = info->connectionType() ? info->connectionType() : "USB";

        OBMultiDeviceSyncConfig cfg = devices_[i]->getMultiDeviceSyncConfig();
        // All devices HARDWARE_TRIGGERING (external HW trigger signal)
        cfg.syncMode         = OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING;
        cfg.triggerOutEnable = false;
        cfg.depthDelayUs         = 0;
        cfg.colorDelayUs         = 0;
        cfg.trigger2ImageDelayUs = 0;
        cfg.triggerOutDelayUs    = 0;
        cfg.framesPerTrigger     = 1;
        devices_[i]->setMultiDeviceSyncConfig(cfg);

        // Enable FPS boost in hardware trigger mode (OB_PROP_FPS_BOOST_BOOL = 275)
        // 仅对支持该属性的设备生效(如 Gemini 305g 不支持,跳过避免抛异常)
        bool fpsBoost = false;
        if (devices_[i]->isPropertySupported(OB_PROP_FPS_BOOST_BOOL, OB_PERMISSION_WRITE)) {
            devices_[i]->setBoolProperty(OB_PROP_FPS_BOOST_BOOL, true);
            fpsBoost = devices_[i]->getBoolProperty(OB_PROP_FPS_BOOST_BOOL);
        }

        auto sn = info->serialNumber();
        std::cout << "Device " << i << " (SN=" << sn
                  << "  type=" << connType << "): HARDWARE_TRIGGERING"
                  << "  triggerOut=" << (cfg.triggerOutEnable ? "true" : "false") << std::endl;
    }

    std::cout << "\nVerify config:" << std::endl;
    for (size_t i = 0; i < devices_.size(); i++) {
        auto check = devices_[i]->getMultiDeviceSyncConfig();
        std::cout << "  Device " << i << " syncMode=" << check.syncMode
                  << "  triggerOut=" << check.triggerOutEnable << std::endl;
    }
}

void DataCollector::resetTimestampAndSyncClock() {
    // HARDWARE_TRIGGERING mode: trigger comes from external HW signal,
    // no timestamp reset needed. Only do per-device clock sync with host.

    // Per-device one-shot clock sync (FAE recommended over enableDeviceClockSync)
    for (auto &dev : devices_) {
        dev->timerSyncWithHost();
    }
    std::cout << "Per-device timer sync completed (" << devices_.size() << " devices)" << std::endl;

    // 使能全局时间戳(global timestamp)：仅对支持的设备开启。
    // 开启后 frame->globalTimeStampUs() 会返回换算到主机时钟域的时间戳，
    // 可用于跨设备时间戳对齐(消除各相机本地晶振漂移)。
    for (size_t i = 0; i < devices_.size(); i++) {
        if (devices_[i]->isGlobalTimestampSupported()) {
            devices_[i]->enableGlobalTimestamp(true);
            std::cout << "Device " << i << ": global timestamp ENABLED" << std::endl;
        } else {
            std::cout << "Device " << i << ": global timestamp NOT supported, skip enable" << std::endl;
        }
    }
}

// 把 color 帧转换为 cv::Mat(BGR)，格式转换逻辑参考官方示例
static cv::Mat frameToMatColor(const std::shared_ptr<ob::Frame>& frame) {
    if (!frame) return cv::Mat();
    auto videoFrame = frame->as<const ob::VideoFrame>();
    if (!videoFrame) return cv::Mat();

    cv::Mat rstMat;
    switch (videoFrame->getFormat()) {
    case OB_FORMAT_MJPG: {
        cv::Mat rawMat(1, videoFrame->getDataSize(), CV_8UC1, videoFrame->getData());
        rstMat = cv::imdecode(rawMat, 1);
    } break;
    case OB_FORMAT_NV21: {
        cv::Mat rawMat(videoFrame->getHeight() * 3 / 2, videoFrame->getWidth(), CV_8UC1, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_NV21);
    } break;
    case OB_FORMAT_YUYV:
    case OB_FORMAT_YUY2: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC2, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_YUY2);
    } break;
    case OB_FORMAT_BGR: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC3, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_BGR2RGB);
    } break;
    case OB_FORMAT_RGB: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC3, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_RGB2BGR);
    } break;
    case OB_FORMAT_RGBA: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC4, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_RGBA2BGR);
    } break;
    case OB_FORMAT_BGRA: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC4, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_BGRA2RGB);
    } break;
    case OB_FORMAT_UYVY: {
        cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC2, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
    } break;
    case OB_FORMAT_I420: {
        cv::Mat rawMat(videoFrame->getHeight() * 3 / 2, videoFrame->getWidth(), CV_8UC1, videoFrame->getData());
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_I420);
    } break;
    default:
        break;
    }
    return rstMat;
}

void DataCollector::saveColorImage(const std::shared_ptr<ob::Frame>& colorFrame, int camIndex) {
    if (!savingEnabled_) return;  // 预热阶段不落盘, 避免垃圾帧污染 PNG/CSV
    cv::Mat mat = frameToMatColor(colorFrame);
    if (mat.empty()) return;

    PendingImage img;
    img.mat = std::move(mat);

    {
        std::lock_guard<std::mutex> lock(*mutexes_[camIndex][1]);
        int seq = savedCount_[camIndex]++;
        uint64_t ts = colorFrame->timeStampUs();
        char fname[128];
        std::snprintf(fname, sizeof(fname), "Device%d_frame_%06d_%llu.png", camIndex, seq,
                      static_cast<unsigned long long>(ts));
        img.fname = fname;
        img.deviceIndex = camIndex;
        img.deviceTimestampUs = ts;
    }
    {
        std::lock_guard<std::mutex> lock(csvMutex_);
        img.groupId = globalSeq_++;
        img.deviceSN = devices_[camIndex]->getDeviceInfo()->serialNumber();
    }

    // 慢 I/O(imwrite + CSV flush) 交给后台线程, 这里只入队, 保证回调线程不被阻塞
    {
        std::lock_guard<std::mutex> lock(imageQueueMutex_);
        imageQueue_.push(std::move(img));
    }
    imageQueueCv_.notify_one();
}

void DataCollector::writerLoop() {
    std::unique_lock<std::mutex> lock(imageQueueMutex_);
    while (true) {
        imageQueueCv_.wait(lock, [this] { return !imageQueue_.empty() || !writerRunning_.load(); });
        while (!imageQueue_.empty()) {
            PendingImage img = std::move(imageQueue_.front());
            imageQueue_.pop();
            lock.unlock();

            cv::imwrite(outputDir_ + "/" + img.fname, img.mat);
            {
                std::lock_guard<std::mutex> csvLock(csvMutex_);
                csvFile_ << img.groupId << "," << img.deviceIndex << ","
                         << img.deviceSN << "," << img.deviceTimestampUs << "," << img.fname << "\n";
                csvFile_.flush();
            }

            lock.lock();
        }
        if (!writerRunning_.load()) break;
    }
}

void DataCollector::collectFrames(const Config& cfg) {
    int deviceCount = static_cast<int>(devices_.size());

    recordingEnabled_ = true;  // 每次采集前复位(收尾会置 false)

    pipelines_.resize(deviceCount);
    allFrames_.resize(deviceCount);
    mutexes_.resize(deviceCount);
    for (int i = 0; i < deviceCount; i++) {
        allFrames_[i].resize(2);
        mutexes_[i].resize(2);
        for (int j = 0; j < 2; j++) {
            mutexes_[i][j] = std::make_shared<std::mutex>();
        }
    }

    // 图像输出：创建目录 + timestamps.csv
    if (!cfg.outputDir.empty()) {
        outputDir_ = cfg.outputDir;
        MKDIR(outputDir_.c_str());
        csvFile_.open(outputDir_ + "/timestamps.csv");
        if (csvFile_.is_open()) {
            csvFile_ << "groupId,deviceIndex,deviceSN,deviceTimestampUs,fileName\n";
            csvFile_.flush();
        } else {
            std::cerr << "[WARN] cannot open timestamps.csv in " << outputDir_ << std::endl;
        }
        savedCount_.assign(deviceCount, 0);
        // 启动后台写盘线程: 慢 I/O(imwrite/CSV flush) 移出回调线程, 避免阻塞 SDK 收帧导致丢帧
        writerRunning_ = true;
        writerThread_ = std::thread(&DataCollector::writerLoop, this);
        std::cout << "Image saving enabled -> " << outputDir_ << std::endl;
    }

    std::vector<std::shared_ptr<ob::Config>> streamCfgs(deviceCount);

    for (int i = 0; i < deviceCount; i++) {
        pipelines_[i] = std::make_shared<ob::Pipeline>(devices_[i]);
        auto streamCfg = std::make_shared<ob::Config>();

        if (cfg.useDepth)
            streamCfg->enableVideoStream(OB_STREAM_DEPTH,
                static_cast<int>(cfg.width), static_cast<int>(cfg.height),
                static_cast<int>(cfg.fps), OB_FORMAT_Y16);
        if (cfg.useColor)
            streamCfg->enableVideoStream(OB_STREAM_COLOR,
                static_cast<int>(cfg.width), static_cast<int>(cfg.height),
                static_cast<int>(cfg.fps), OB_FORMAT_YUYV);

        streamCfgs[i] = streamCfg;
    }

    // 外部触发自动控制: 先关触发, 确保三台相机从"无触发"状态一起 arm
    if (cfg.triggerHz > 0) {
        writeTriggerFramerate(0);
    }

    // ---- 阶段2: 3 线程 + 主线程发令枪, 同时 start ----
    // 数据采集本身无共享写(每台相机写自己的 allFrames_[i][0/1]), 故不加数据锁;
    // 下面这对 mutex + condition_variable 仅用于"发令枪"通知, 不保护数据。
    {
        std::mutex              goMutex;
        std::condition_variable goCv;
        bool                    go = false;
        std::vector<std::exception_ptr> errs(deviceCount);

        std::vector<std::thread> startThreads;
        startThreads.reserve(deviceCount);
        for (int i = 0; i < deviceCount; i++) {
            int camIndex = i;
            startThreads.emplace_back([this, camIndex, cfg, &streamCfgs, &goMutex, &goCv, &go, &errs]() {
                try {
                    std::unique_lock<std::mutex> lock(goMutex);
                    goCv.wait(lock, [&go] { return go; });
                    pipelines_[camIndex]->start(streamCfgs[camIndex],
                        [this, camIndex](std::shared_ptr<ob::FrameSet> frameSet) {
                            
                            if (!recordingEnabled_.load()) return;  // 收尾冻结后直接丢弃新帧

                            auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                            if (colorFrame) {
                                FrameStamp fs;
                                fs.hwTimestampUs     = colorFrame->timeStampUs();
                                fs.globalTimestampUs = colorFrame->globalTimeStampUs();
                                fs.sysTimestampUs    = colorFrame->systemTimeStampUs();
                                fs.frameNumber       = static_cast<int64_t>(colorFrame->getIndex()); // SDK 每流递增帧序号
                                fs.deviceIndex       = camIndex;
                                fs.streamType        = StreamType::COLOR;
                                {
                                    std::lock_guard<std::mutex> lock(*mutexes_[camIndex][1]);
                                    allFrames_[camIndex][1].push_back(fs);
                                }
                                if (!outputDir_.empty()) {
                                    saveColorImage(colorFrame, camIndex);
                                }
                            }

                            auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                            if (depthFrame) {
                                FrameStamp fs;
                                fs.hwTimestampUs     = depthFrame->timeStampUs();
                                fs.globalTimestampUs = depthFrame->globalTimeStampUs();
                                fs.sysTimestampUs    = depthFrame->systemTimeStampUs();
                                fs.frameNumber       = static_cast<int64_t>(depthFrame->getIndex()); // SDK 每流递增帧序号
                                fs.deviceIndex       = camIndex;
                                fs.streamType        = StreamType::DEPTH;
                                std::lock_guard<std::mutex> lock(*mutexes_[camIndex][0]);
                                allFrames_[camIndex][0].push_back(fs);
                            }
                        });

                    auto sn = devices_[camIndex]->getDeviceInfo()->serialNumber();
                    std::cout << "Device " << camIndex << " (SN=" << sn << ") pipeline started: "
                              << (cfg.useDepth ? "Depth " : "")
                              << (cfg.useColor ? "Color " : "")
                              << cfg.width << "x" << cfg.height << " @ " << cfg.fps << "fps"
                              << std::endl;
                } catch (...) {
                    errs[camIndex] = std::current_exception();
                }
            });
        }

        // 发令枪: 所有线程已就位等待, 统一放行
        {
            std::lock_guard<std::mutex> lock(goMutex);
            go = true;
        }
        goCv.notify_all();

        for (auto& t : startThreads) {
            t.join();
        }
        for (auto& e : errs) {
            if (e) std::rethrow_exception(e);
        }
    }

    // 三台已 arm 就绪。短暂 settle 后开启触发 —— 此刻是"发令枪", 三台从同一触发沿出帧
    if (cfg.triggerHz > 0) {
        // 本次会做 color 预热: 先禁止落盘, 预热结束(清空预热帧)时再恢复, 避免垃圾帧写入 PNG/CSV
        if (cfg.useColor) savingEnabled_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        writeTriggerFramerate(static_cast<int>(cfg.triggerHz));
    }

    // 预热: color 传感器冷启动需约 300~400ms 跑完自动曝光/白平衡, 这期间不出帧,
    // 导致 color 比 depth 固定少 3~4 帧(实测 raw.csv 首帧差 294~393ms, 中间无丢帧)。
    // 固定 sleep(500ms) 太临界且靠猜时间; 这里改为"条件等待": 轮询每台设备 color
    // 是否已各收到 >= WARMUP_MIN_COLOR 帧(即已稳定出帧), 收到后统一清空预热帧再开落盘,
    // 使 depth 与 color 从同一干净起点计数。兜底最多等 WARMUP_TIMEOUT_MS, 超时也清空开始。
    if (cfg.useColor && cfg.triggerHz > 0) {
        const int  WARMUP_MIN_COLOR = 3;
        const auto WARMUP_TIMEOUT  = std::chrono::milliseconds(2000);

        auto colorReady = [&]() {
            for (int i = 0; i < deviceCount; i++) {
                std::lock_guard<std::mutex> lock(*mutexes_[i][1]);
                if (static_cast<int>(allFrames_[i][1].size()) < WARMUP_MIN_COLOR) return false;
            }
            return true;
        };

        auto warmupStart = std::chrono::steady_clock::now();
        while (!colorReady() &&
               std::chrono::steady_clock::now() - warmupStart < WARMUP_TIMEOUT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // 清空预热阶段积累的所有帧(含 depth), 让 depth 与 color 从同一干净起点开始
        for (int i = 0; i < deviceCount; i++) {
            for (int j = 0; j < 2; j++) {
                std::lock_guard<std::mutex> lock(*mutexes_[i][j]);
                allFrames_[i][j].clear();
            }
        }

        // 预热结束, 之后 color 帧正常落盘(PNG + CSV 从 groupId=0 / seq=0 干净起步)
        savingEnabled_ = true;
        std::cout << "[WARMUP] color ready (or timeout), pre-trigger frames cleared" << std::endl;
    }

    std::cout << "\nCollecting frames for " << cfg.durationSec
              << " seconds... (Ctrl+C to stop early)" << std::endl;
    auto startTime = std::chrono::steady_clock::now();
    while (running_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed >= cfg.durationSec) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ---- 收尾: 先冻结计数, 再在触发仍开时逐台 stop, 最后关触发 ----
    // 1) 冻结计数: recordingEnabled_=false 后回调直接丢弃新帧, 不再 push_back/落盘,
    //    因此 stop 期间即使触发仍在跑、相机仍出帧, 帧数也不会漂移(对应原 104/101/98 问题)。
    // 2) 触发仍开时 stop: 若先关触发再 stop, 收流通道等不到下一帧, 2.5s 超时后触发
    //    tegra_camera 驱动的 use-after-free bug, 板子 panic 重启(见 pstore 日志)。
    //    触发开着时 stop(), 收帧线程有帧可收, 能正常返回并发出 STREAMOFF, 通道干净关闭。
    // 3) 全部 stop 完成(通道已关)后再关触发, 不会再出现饿死/超时。
    recordingEnabled_ = false;

    for (int i = deviceCount - 1; i >= 0; i--) {
        std::cout << "Stopping pipeline " << i << " ..." << std::endl;
        pipelines_[i]->stop();
        std::cout << "Pipeline " << i << " stopped" << std::endl;
    }

    if (cfg.triggerHz > 0) {
        writeTriggerFramerate(0);
    }

    if (!outputDir_.empty()) {
        // 停止后台写盘线程: 置停止标志 → 唤醒 → 等待队列清空并退出, 再关 CSV
        {
            std::lock_guard<std::mutex> lock(imageQueueMutex_);
            writerRunning_ = false;
        }
        imageQueueCv_.notify_all();
        if (writerThread_.joinable()) writerThread_.join();

        if (csvFile_.is_open()) {
            csvFile_.close();
        }
        int totalSaved = 0;
        for (int i = 0; i < deviceCount; i++) {
            totalSaved += savedCount_[i];
        }
        std::cout << "Images saved: " << totalSaved << " frames to " << outputDir_
                  << " (timestamps.csv written)" << std::endl;
    }

    int totalFrames = 0;
    for (int i = 0; i < deviceCount; i++) {
        int depthCount = static_cast<int>(allFrames_[i][0].size());
        int colorCount = static_cast<int>(allFrames_[i][1].size());
        std::cout << "Device " << i << "  Depth frames: " << depthCount
                  << "  Color frames: " << colorCount << std::endl;
        totalFrames += depthCount + colorCount;
    }
    std::cout << "Total frames: " << totalFrames << std::endl;
}

void DataCollector::exportRawCSV(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "[WARN] cannot open raw CSV: " << path << std::endl;
        return;
    }
    f << "deviceIndex,streamType,hwTimestampUs,globalTimestampUs,sysTimestampUs,frameNumber\n";
    int count = 0;
    for (size_t i = 0; i < allFrames_.size(); i++) {
        for (size_t s = 0; s < allFrames_[i].size(); s++) {
            for (const auto& fs : allFrames_[i][s]) {
                f << fs.deviceIndex << ","
                  << (fs.streamType == StreamType::DEPTH ? "DEPTH" : "COLOR") << ","
                  << fs.hwTimestampUs << ","
                  << fs.globalTimestampUs << ","
                  << fs.sysTimestampUs << ","
                  << fs.frameNumber << "\n";
                count++;
            }
        }
    }
    f.close();
    std::cout << "Raw CSV exported: " << path << " (" << count << " frames)" << std::endl;
}