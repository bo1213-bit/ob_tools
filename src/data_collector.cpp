// data_collector.cpp
// 模块1: 数据采集

#include "data_collector.h"

#include <chrono>
#include <cstdio>
#include <iostream>
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

void DataCollector::run(const Config& cfg) {
    running_ = true;
    std::cout << "=== DataCollector: Start ===" << std::endl;
    std::cout << "Config: " << cfg.width << "x" << cfg.height
              << " @ " << cfg.fps << "fps"
              << "  duration=" << cfg.durationSec << "s"
              << "  depth=" << (cfg.useDepth ? "on" : "off")
              << "  color=" << (cfg.useColor ? "on" : "off") << std::endl;

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

        // // Enable FPS boost in hardware trigger mode (OB_PROP_FPS_BOOST_BOOL = 275)
        // // 仅对支持该属性的设备生效(如 Gemini 305g 不支持,跳过避免抛异常)
        // bool fpsBoost = false;
        // if (devices_[i]->isPropertySupported(OB_PROP_FPS_BOOST_BOOL, OB_PERMISSION_WRITE)) {
        //     devices_[i]->setBoolProperty(OB_PROP_FPS_BOOST_BOOL, true);
        //     fpsBoost = devices_[i]->getBoolProperty(OB_PROP_FPS_BOOST_BOOL);
        // }

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
    cv::Mat mat = frameToMatColor(colorFrame);
    if (mat.empty()) return;

    int seq;
    {
        std::lock_guard<std::mutex> lock(*mutexes_[camIndex][1]);
        seq = savedCount_[camIndex]++;
    }

    uint64_t ts = colorFrame->timeStampUs();
    char fname[128];
    std::snprintf(fname, sizeof(fname), "Device%d_frame_%06d_%llu.png", camIndex, seq,
                  static_cast<unsigned long long>(ts));

    cv::imwrite(outputDir_ + "/" + fname, mat);

    {
        std::lock_guard<std::mutex> lock(csvMutex_);
        csvFile_ << globalSeq_++ << "," << camIndex << ","
                 << devices_[camIndex]->getDeviceInfo()->serialNumber() << "," << ts << "," << fname << "\n";
        csvFile_.flush();
    }
}

void DataCollector::collectFrames(const Config& cfg) {
    int deviceCount = static_cast<int>(devices_.size());

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
        std::cout << "Image saving enabled -> " << outputDir_ << std::endl;
    }

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

        int camIndex = i;
        pipelines_[i]->start(streamCfg, [this, camIndex, cfg](std::shared_ptr<ob::FrameSet> frameSet) {
            auto nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            if (cfg.useDepth) {
                auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                if (depthFrame) {
                    FrameStamp fs;
                    fs.hwTimestampUs     = depthFrame->timeStampUs();
                    fs.globalTimestampUs = depthFrame->globalTimeStampUs();
                    fs.sysTimestampUs    = depthFrame->systemTimeStampUs();
                    fs.frameNumber       = 0; // 时间戳已包含足够信息
                    fs.deviceIndex       = camIndex;
                    fs.streamType        = StreamType::DEPTH;
                    std::lock_guard<std::mutex> lock(*mutexes_[camIndex][0]);
                    allFrames_[camIndex][0].push_back(fs);
                }
            }

            if (cfg.useColor) {
                auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                if (colorFrame) {
                    FrameStamp fs;
                    fs.hwTimestampUs     = colorFrame->timeStampUs();
                    fs.globalTimestampUs = colorFrame->globalTimeStampUs();
                    fs.sysTimestampUs    = colorFrame->systemTimeStampUs();
                    fs.frameNumber       = 0;
                    fs.deviceIndex       = camIndex;
                    fs.streamType        = StreamType::COLOR;
                    {
                        std::lock_guard<std::mutex> lock(*mutexes_[camIndex][1]);
                        allFrames_[camIndex][1].push_back(fs);
                    }
                    // 保存 color 图像到文件（输出目录非空时）
                    if (!outputDir_.empty()) {
                        saveColorImage(colorFrame, camIndex);
                    }
                }
            }
        });

        auto sn = devices_[i]->getDeviceInfo()->serialNumber();
        std::cout << "Device " << i << " (SN=" << sn << ") pipeline started: "
                  << (cfg.useDepth ? "Depth " : "")
                  << (cfg.useColor ? "Color " : "")
                  << cfg.width << "x" << cfg.height << " @ " << cfg.fps << "fps" << std::endl;
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

    for (int i = deviceCount - 1; i >= 0; i--) {
        pipelines_[i]->stop();
    }
    std::cout << "Pipelines stopped" << std::endl;

    if (!outputDir_.empty()) {
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
    f << "deviceIndex,streamType,hwTimestampUs,globalTimestampUs,sysTimestampUs\n";
    int count = 0;
    for (size_t i = 0; i < allFrames_.size(); i++) {
        for (size_t s = 0; s < allFrames_[i].size(); s++) {
            for (const auto& fs : allFrames_[i][s]) {
                f << fs.deviceIndex << ","
                  << (fs.streamType == StreamType::DEPTH ? "DEPTH" : "COLOR") << ","
                  << fs.hwTimestampUs << ","
                  << fs.globalTimestampUs << ","
                  << fs.sysTimestampUs << "\n";
                count++;
            }
        }
    }
    f.close();
    std::cout << "Raw CSV exported: " << path << " (" << count << " frames)" << std::endl;
}