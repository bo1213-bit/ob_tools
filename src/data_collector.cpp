// data_collector.cpp
// 模块1: 数据采集

#include "data_collector.h"

#include <algorithm>
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

    const std::vector<std::string> officialOrder = {
        "CPBG1630011D",
        "CV3T561000H0",
        "CV3T561000B5",
    };
    std::stable_sort(devices_.begin(), devices_.end(), [&](const auto& lhs, const auto& rhs) {
        const std::string lhsSn = lhs->getDeviceInfo()->serialNumber();
        const std::string rhsSn = rhs->getDeviceInfo()->serialNumber();
        auto lhsIt = std::find(officialOrder.begin(), officialOrder.end(), lhsSn);
        auto rhsIt = std::find(officialOrder.begin(), officialOrder.end(), rhsSn);
        return lhsIt < rhsIt;
    });

    std::cout << "\nOfficial stream order:" << std::endl;
    for (size_t i = 0; i < devices_.size(); ++i) {
        std::cout << "  Device " << i << ": SN="
                  << devices_[i]->getDeviceInfo()->serialNumber() << std::endl;
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
        cfg.triggerOutEnable = true;
        cfg.depthDelayUs         = 0;
        cfg.colorDelayUs         = 0;
        cfg.trigger2ImageDelayUs = 0;
        cfg.triggerOutDelayUs    = 0;
        cfg.framesPerTrigger     = 1;
        devices_[i]->setMultiDeviceSyncConfig(cfg);

        // Gemini 335Lg needs FPS boost to deliver one frame for every trigger.
        // Explicitly set it on each run because the device retains this property.
        const bool fpsBoostSupported =
            devices_[i]->isPropertySupported(OB_PROP_FPS_BOOST_BOOL, OB_PERMISSION_WRITE);
        bool fpsBoost = false;
        if (fpsBoostSupported) {
            devices_[i]->setBoolProperty(OB_PROP_FPS_BOOST_BOOL, true);
            fpsBoost = devices_[i]->getBoolProperty(OB_PROP_FPS_BOOST_BOOL);
        }
        std::cout << "  FPS boost: "
                  << (fpsBoostSupported ? (fpsBoost ? "enabled" : "disabled") : "unsupported")
                  << "  requested=true" << std::endl;

        auto sn = info->serialNumber();
        std::cout << "Device " << i << " (SN=" << sn
                  << "  type=" << connType << "): HARDWARE_TRIGGERING"
                  << "  triggerOut=" << (cfg.triggerOutEnable ? "true" : "false") << std::endl;
    }

    std::cout << "\nVerify config:" << std::endl;
    for (size_t i = 0; i < devices_.size(); i++) {
        auto check = devices_[i]->getMultiDeviceSyncConfig();
        std::cout << "  Device " << i
                  << " syncMode=" << check.syncMode
                  << " triggerOut=" << check.triggerOutEnable
                  << " depthDelayUs=" << check.depthDelayUs
                  << " colorDelayUs=" << check.colorDelayUs
                  << " trigger2ImageDelayUs=" << check.trigger2ImageDelayUs
                  << " triggerOutDelayUs=" << check.triggerOutDelayUs
                  << " framesPerTrigger=" << check.framesPerTrigger
                  << std::endl;
    }
}

void DataCollector::resetTimestampAndSyncClock() {
    // Follow the official multi-device synchronization order exactly:
    // enable global timestamps first, then synchronize device clocks, and allow
    // the global-timestamp conversion to settle before starting pipelines.
    for (size_t i = 0; i < devices_.size(); i++) {
        if (devices_[i]->isGlobalTimestampSupported()) {
            devices_[i]->enableGlobalTimestamp(true);
            std::cout << "Device " << i << ": global timestamp ENABLED" << std::endl;
        } else {
            std::cout << "Device " << i << ": global timestamp NOT supported, skip enable" << std::endl;
        }
    }

    std::cout << "Syncing device clocks..." << std::endl;
    context_->enableDeviceClockSync(0);
    std::cout << "Device clocks synced (" << devices_.size() << " devices)" << std::endl;

    constexpr auto GLOBAL_TIMESTAMP_SETTLE_TIME = std::chrono::seconds(1);
    std::cout << "Waiting " << GLOBAL_TIMESTAMP_SETTLE_TIME.count()
              << "s for global timestamps to stabilize..." << std::endl;
    std::this_thread::sleep_for(GLOBAL_TIMESTAMP_SETTLE_TIME);
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
    if (!savingEnabled_ || !recordingEnabled_) return;  // 正式记录窗口外不落盘，避免启动期帧污染 PNG/CSV
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

void DataCollector::recordFrameDiagnostics(int camIndex, int streamIndex,
                                           const FrameStamp& stamp,
                                           int64_t metadataFrameNumber,
                                           int64_t callbackUs,
                                           int64_t expectedPeriodUs) {
    const int64_t armUs = recordingArmSteadyUs_.load();
    if (armUs == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(*diagnosticMutexes_[camIndex]);
    auto& stream = diagnostics_[camIndex].streams[streamIndex];
    const int64_t callbackOffsetUs = callbackUs - armUs;

    if (!stream.first.valid) {
        stream.first = {true, stamp, metadataFrameNumber, callbackOffsetUs};
    }
    stream.last = {true, stamp, metadataFrameNumber, callbackOffsetUs};
    stream.acceptedFrames++;

    if (stream.hasPrevious) {
        const int64_t indexDelta = stamp.frameNumber - stream.previous.frameNumber;
        if (indexDelta > 1) {
            stream.indexGapEvents++;
            stream.missingIndexFrames += static_cast<uint64_t>(indexDelta - 1);
        } else if (indexDelta <= 0) {
            stream.indexRegressions++;
        }

        if (metadataFrameNumber < 0) {
            stream.missingMetadataFrames++;
        } else if (stream.previousMetadataFrameNumber >= 0) {
            const int64_t metadataDelta = metadataFrameNumber - stream.previousMetadataFrameNumber;
            if (metadataDelta > 1) {
                stream.metadataGapEvents++;
                stream.missingMetadataFrames += static_cast<uint64_t>(metadataDelta - 1);
            } else if (metadataDelta <= 0) {
                stream.metadataRegressions++;
            }
        }

        if (stamp.hwTimestampUs < stream.previous.hwTimestampUs) {
            stream.hwTimestampRegressions++;
        }
        if (stamp.globalTimestampUs < stream.previous.globalTimestampUs) {
            stream.globalTimestampRegressions++;
        }
        if (stamp.sysTimestampUs < stream.previous.sysTimestampUs) {
            stream.systemTimestampRegressions++;
        }

        const int64_t cadenceLimitUs = expectedPeriodUs + expectedPeriodUs / 2;
        if (stamp.hwTimestampUs - stream.previous.hwTimestampUs > cadenceLimitUs) {
            stream.hwCadenceGaps++;
        }
        if (stamp.globalTimestampUs - stream.previous.globalTimestampUs > cadenceLimitUs) {
            stream.globalCadenceGaps++;
        }
        if (stamp.sysTimestampUs - stream.previous.sysTimestampUs > cadenceLimitUs) {
            stream.systemCadenceGaps++;
        }
        if (callbackUs - stream.previousCallbackUs > cadenceLimitUs) {
            stream.callbackCadenceGaps++;
        }
    } else if (metadataFrameNumber < 0) {
        stream.missingMetadataFrames++;
    }

    stream.hasPrevious = true;
    stream.previous = stamp;
    stream.previousMetadataFrameNumber = metadataFrameNumber;
    stream.previousCallbackUs = callbackUs;
}

void DataCollector::printDiagnosticsSummary(const Config& cfg, int64_t expectedPeriodUs) const {
    std::cout << "\n=== Collection Diagnostics (observation only) ===" << std::endl;
    std::cout << "Expected stream period: " << expectedPeriodUs << "us ("
              << cfg.fps << " fps), cadence-gap threshold: "
              << expectedPeriodUs + expectedPeriodUs / 2 << "us" << std::endl;

    const char* streamNames[] = {"Depth", "Color"};
    for (size_t device = 0; device < diagnostics_.size(); ++device) {
        std::lock_guard<std::mutex> lock(*diagnosticMutexes_[device]);
        const auto& diag = diagnostics_[device];
        const char* serial = device < devices_.size()
                                 ? devices_[device]->getDeviceInfo()->serialNumber()
                                 : "unknown";

        std::cout << "Device " << device << " (SN=" << serial << "): callbacks closed="
                  << diag.closedGateCallbacks << " open=" << diag.openGateCallbacks
                  << ", framesets complete=" << diag.completeFrameSets
                  << " depth-only=" << diag.depthOnlyFrameSets
                  << " color-only=" << diag.colorOnlyFrameSets
                  << " empty=" << diag.emptyFrameSets << std::endl;

        for (int streamIndex = 0; streamIndex < 2; ++streamIndex) {
            const auto& stream = diag.streams[streamIndex];
            std::cout << "  " << streamNames[streamIndex]
                      << ": accepted=" << stream.acceptedFrames
                      << " index(gaps=" << stream.indexGapEvents
                      << ", missing=" << stream.missingIndexFrames
                      << ", regressions=" << stream.indexRegressions << ")"
                      << " metadata(gaps=" << stream.metadataGapEvents
                      << ", missing=" << stream.missingMetadataFrames
                      << ", regressions=" << stream.metadataRegressions << ")"
                      << " timestamp-regressions(hw/global/sys)="
                      << stream.hwTimestampRegressions << "/"
                      << stream.globalTimestampRegressions << "/"
                      << stream.systemTimestampRegressions
                      << " cadence-gaps(hw/global/sys/callback)="
                      << stream.hwCadenceGaps << "/"
                      << stream.globalCadenceGaps << "/"
                      << stream.systemCadenceGaps << "/"
                      << stream.callbackCadenceGaps << std::endl;

            if (stream.first.valid) {
                std::cout << "    first: index=" << stream.first.stamp.frameNumber
                          << " metadata=" << stream.first.metadataFrameNumber
                          << " hw/global/sys=" << stream.first.stamp.hwTimestampUs << "/"
                          << stream.first.stamp.globalTimestampUs << "/"
                          << stream.first.stamp.sysTimestampUs
                          << " callback-after-arm=" << stream.first.callbackOffsetUs << "us"
                          << std::endl;
                std::cout << "    last:  index=" << stream.last.stamp.frameNumber
                          << " metadata=" << stream.last.metadataFrameNumber
                          << " hw/global/sys=" << stream.last.stamp.hwTimestampUs << "/"
                          << stream.last.stamp.globalTimestampUs << "/"
                          << stream.last.stamp.sysTimestampUs
                          << " callback-after-arm=" << stream.last.callbackOffsetUs << "us"
                          << std::endl;
            }
            if (stream.profileObserved) {
                std::cout << "    observed profile=" << stream.observedWidth << "x"
                          << stream.observedHeight << " format=" << stream.observedFormat
                          << std::endl;
            }
        }
    }
    std::cout << "==============================================" << std::endl;
}

void DataCollector::collectFrames(const Config& cfg) {
    const int deviceCount = static_cast<int>(devices_.size());

    pipelines_.resize(deviceCount);
    allFrames_.resize(deviceCount);
    mutexes_.resize(deviceCount);
    diagnostics_.assign(deviceCount, DeviceDiagnostics{});
    diagnosticMutexes_.resize(deviceCount);

    for (int i = 0; i < deviceCount; i++) {
        allFrames_[i].resize(2);
        mutexes_[i].resize(2);
        diagnosticMutexes_[i] = std::make_shared<std::mutex>();
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
        writerRunning_ = true;
        writerThread_ = std::thread(&DataCollector::writerLoop, this);
        std::cout << "Image saving enabled -> " << outputDir_ << std::endl;
    }

    std::vector<std::shared_ptr<ob::Config>> streamCfgs(deviceCount);
    for (int i = 0; i < deviceCount; i++) {
        pipelines_[i] = std::make_shared<ob::Pipeline>(devices_[i]);
        auto streamCfg = std::make_shared<ob::Config>();

        if (cfg.useDepth) {
            streamCfg->enableVideoStream(OB_STREAM_DEPTH,
                static_cast<int>(cfg.width), static_cast<int>(cfg.height),
                static_cast<int>(cfg.fps), OB_FORMAT_Y16);
        }
        if (cfg.useColor) {
            streamCfg->enableVideoStream(OB_STREAM_COLOR,
                static_cast<int>(cfg.width), static_cast<int>(cfg.height),
                static_cast<int>(cfg.fps), OB_FORMAT_YUYV);
        }

        streamCfgs[i] = streamCfg;
    }

    const int64_t expectedPeriodUs = std::max<int64_t>(1, 1000000 / cfg.fps);
    recordingEnabled_ = false;
    savingEnabled_ = false;
    recordingArmSteadyUs_ = 0;

    std::vector<bool> readyDepth(deviceCount, !cfg.useDepth);
    std::vector<bool> readyColor(deviceCount, !cfg.useColor);
    std::mutex readyMutex;
    std::condition_variable readyCv;
    auto allDevicesReady = [&]() {
        for (int i = 0; i < deviceCount; ++i) {
            if (!readyDepth[i] || !readyColor[i]) {
                return false;
            }
        }
        return true;
    };

    // 按官方样例的出流方式：不要在 start 前关闭触发。硬件触发模式下，
    // pipeline->start() 期间就必须有 trigger，否则早启动的 pipeline 可能一直不回调。
    if (cfg.triggerHz > 0) {
        writeTriggerFramerate(static_cast<int>(cfg.triggerHz));
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    for (int camIndex = 0; camIndex < deviceCount; ++camIndex) {
        try {
            pipelines_[camIndex]->start(streamCfgs[camIndex],
                [this, camIndex, cfg, expectedPeriodUs,
                 &readyDepth, &readyColor, &readyMutex, &readyCv](std::shared_ptr<ob::FrameSet> frameSet) {
                    const int64_t callbackUs = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();

                    auto colorFrame = frameSet ? frameSet->getFrame(OB_FRAME_COLOR) : nullptr;
                    auto depthFrame = frameSet ? frameSet->getFrame(OB_FRAME_DEPTH) : nullptr;

                    if (depthFrame || colorFrame) {
                        {
                            std::lock_guard<std::mutex> readyLock(readyMutex);
                            if (depthFrame) readyDepth[camIndex] = true;
                            if (colorFrame) readyColor[camIndex] = true;
                        }
                        readyCv.notify_all();
                    }

                    const bool gateOpen = recordingEnabled_.load();
                    {
                        std::lock_guard<std::mutex> lock(*diagnosticMutexes_[camIndex]);
                        auto& diag = diagnostics_[camIndex];
                        if (gateOpen) {
                            diag.openGateCallbacks++;
                        } else {
                            diag.closedGateCallbacks++;
                        }
                        if (depthFrame && colorFrame) {
                            diag.completeFrameSets++;
                        } else if (depthFrame) {
                            diag.depthOnlyFrameSets++;
                        } else if (colorFrame) {
                            diag.colorOnlyFrameSets++;
                        } else {
                            diag.emptyFrameSets++;
                        }
                    }

                    // start/warmup 阶段只用来确认 SDK 已经实际出帧；正式窗口开启后才写结果。
                    if (!gateOpen) {
                        return;
                    }

                    auto metadataFrameNumber = [](const std::shared_ptr<ob::Frame>& frame) {
                        return frame->hasMetadata(OB_FRAME_METADATA_TYPE_FRAME_NUMBER)
                                   ? static_cast<int64_t>(frame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER))
                                   : int64_t{-1};
                    };
                    auto observeProfile = [this, camIndex](int streamIndex,
                                                           const std::shared_ptr<ob::Frame>& frame) {
                        auto videoFrame = frame->as<const ob::VideoFrame>();
                        if (!videoFrame) return;
                        std::lock_guard<std::mutex> lock(*diagnosticMutexes_[camIndex]);
                        auto& stream = diagnostics_[camIndex].streams[streamIndex];
                        if (!stream.profileObserved) {
                            stream.profileObserved = true;
                            stream.observedWidth = videoFrame->getWidth();
                            stream.observedHeight = videoFrame->getHeight();
                            stream.observedFormat = static_cast<int>(videoFrame->getFormat());
                        }
                    };

                    if (colorFrame) {
                        FrameStamp fs;
                        fs.hwTimestampUs     = colorFrame->timeStampUs();
                        fs.globalTimestampUs = colorFrame->globalTimeStampUs();
                        fs.sysTimestampUs    = colorFrame->systemTimeStampUs();
                        fs.frameNumber       = static_cast<int64_t>(colorFrame->getIndex());
                        fs.deviceIndex       = camIndex;
                        fs.streamType        = StreamType::COLOR;
                        const int64_t metadataNumber = metadataFrameNumber(colorFrame);
                        {
                            std::lock_guard<std::mutex> lock(*mutexes_[camIndex][1]);
                            allFrames_[camIndex][1].push_back(fs);
                        }
                        observeProfile(1, colorFrame);
                        recordFrameDiagnostics(camIndex, 1, fs, metadataNumber, callbackUs,
                                               expectedPeriodUs);
                        if (!outputDir_.empty()) {
                            saveColorImage(colorFrame, camIndex);
                        }
                    }

                    if (depthFrame) {
                        FrameStamp fs;
                        fs.hwTimestampUs     = depthFrame->timeStampUs();
                        fs.globalTimestampUs = depthFrame->globalTimeStampUs();
                        fs.sysTimestampUs    = depthFrame->systemTimeStampUs();
                        fs.frameNumber       = static_cast<int64_t>(depthFrame->getIndex());
                        fs.deviceIndex       = camIndex;
                        fs.streamType        = StreamType::DEPTH;
                        const int64_t metadataNumber = metadataFrameNumber(depthFrame);
                        {
                            std::lock_guard<std::mutex> lock(*mutexes_[camIndex][0]);
                            allFrames_[camIndex][0].push_back(fs);
                        }
                        observeProfile(0, depthFrame);
                        recordFrameDiagnostics(camIndex, 0, fs, metadataNumber, callbackUs,
                                               expectedPeriodUs);
                    }
                });
        }
        catch (ob::Error& e) {
            auto sn = devices_[camIndex]->getDeviceInfo()->serialNumber();
            std::cerr << "starting stream failed: " << sn << std::endl;
            std::cerr << "function:" << e.getName()
                      << "\nargs:" << e.getArgs()
                      << "\nmessage:" << e.what()
                      << "\nstatus:" << e.getStatus()
                      << "\ntype:" << e.getExceptionType() << std::endl;
            continue;
        }

        auto sn = devices_[camIndex]->getDeviceInfo()->serialNumber();
        std::cout << "Device " << camIndex << " (SN=" << sn << ") pipeline started: "
                  << (cfg.useDepth ? "Depth " : "")
                  << (cfg.useColor ? "Color " : "")
                  << cfg.width << "x" << cfg.height << " @ " << cfg.fps << "fps"
                  << std::endl;
    }

    std::cout << "\n[WARMUP] Waiting until every device receives the requested stream(s)..." << std::endl;
    bool ready = false;
    {
        std::unique_lock<std::mutex> readyLock(readyMutex);
        const auto readyDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        while (running_ && !allDevicesReady()) {
            if (readyCv.wait_until(readyLock, readyDeadline) == std::cv_status::timeout) {
                break;
            }
        }
        ready = allDevicesReady();

        for (int i = 0; i < deviceCount; ++i) {
            auto sn = devices_[i]->getDeviceInfo()->serialNumber();
            std::cout << "  Device " << i << " (SN=" << sn << ") ready:"
                      << " depth=" << (readyDepth[i] ? "yes" : "no")
                      << " color=" << (readyColor[i] ? "yes" : "no") << std::endl;
        }
    }

    if (!ready) {
        std::cerr << "[WARN] Not all devices became ready before timeout; recording available streams anyway." << std::endl;
    }

    // 清空 start/warmup 阶段积累的数据和诊断，只保留正式采集窗口的数据。
    for (int i = 0; i < deviceCount; ++i) {
        for (int s = 0; s < 2; ++s) {
            std::lock_guard<std::mutex> lock(*mutexes_[i][s]);
            allFrames_[i][s].clear();
        }
        {
            std::lock_guard<std::mutex> diagLock(*diagnosticMutexes_[i]);
            diagnostics_[i] = DeviceDiagnostics{};
        }
    }
    if (!cfg.outputDir.empty()) {
        std::lock_guard<std::mutex> csvLock(csvMutex_);
        globalSeq_ = 0;
        std::fill(savedCount_.begin(), savedCount_.end(), 0);
    }

    const auto recordingStart = std::chrono::steady_clock::now();
    recordingArmSteadyUs_ = std::chrono::duration_cast<std::chrono::microseconds>(
        recordingStart.time_since_epoch()).count();
    recordingEnabled_ = true;
    savingEnabled_ = true;

    std::cout << "\n[RECORDING] Formal recording window for " << cfg.durationSec
              << " seconds." << std::endl;
    std::cout << "Collecting frames... (Ctrl+C to stop early)" << std::endl;

    const auto recordingDeadline = recordingStart + std::chrono::seconds(cfg.durationSec);
    while (running_ && std::chrono::steady_clock::now() < recordingDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    recordingEnabled_ = false;
    savingEnabled_ = false;

    printDiagnosticsSummary(cfg, expectedPeriodUs);

    for (int i = deviceCount - 1; i >= 0; i--) {
        if (!pipelines_[i]) continue;
        std::cout << "Stopping pipeline " << i << " ..." << std::endl;
        pipelines_[i]->stop();
        std::cout << "Pipeline " << i << " stopped" << std::endl;
    }

    if (cfg.triggerHz > 0) {
        writeTriggerFramerate(0);
    }

    if (!outputDir_.empty()) {
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