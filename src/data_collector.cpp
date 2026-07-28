// data_collector.cpp
// 模块1: 数据采集

#include "data_collector.h"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

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
        if (connType == "GMSL2") {
            // GMSL: 所有相机设为 HARDWARE_TRIGGERING，触发来自外部硬件信号
            // 和 MultiDeviceSync 一致
            cfg.syncMode         = OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING;
            cfg.triggerOutEnable = false;
        } else {
            // USB: Primary/Secondary mode
            if (i == 0) {
                cfg.syncMode         = OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
                cfg.triggerOutEnable = true;
            } else {
                cfg.syncMode         = OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
                cfg.triggerOutEnable = false;
            }
        }
        cfg.depthDelayUs         = 0;
        cfg.colorDelayUs         = 0;
        cfg.trigger2ImageDelayUs = 0;
        cfg.triggerOutDelayUs    = 0;
        cfg.framesPerTrigger     = 1;
        devices_[i]->setMultiDeviceSyncConfig(cfg);

        auto sn = info->serialNumber();
        std::cout << "Device " << i << " (SN=" << sn
                  << "  type=" << connType << "): "
                  << (connType == "GMSL2" ? "HARDWARE_TRIGGERING" : (i == 0 ? "PRIMARY" : "SECONDARY"))
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
    // 检查是否 GMSL 模式
    bool isGmsl = false;
    if (!devices_.empty()) {
        auto info = devices_[0]->getDeviceInfo();
        std::string connType = info->connectionType() ? info->connectionType() : "USB";
        isGmsl = (connType == "GMSL2");
    }

    if (isGmsl) {
        // GMSL: 触发来自外部硬件信号，跳过时间戳复位
        // 和 MultiDeviceSync 一致
        std::cout << "\nGMSL mode: skip timestamp reset (use external HW trigger)."
                  << std::endl;
    } else {
        // USB: PRIMARY 发送时间戳复位信号给所有 SECONDARY
        devices_[0]->setBoolProperty(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, true);
        devices_[0]->setIntProperty(OB_PROP_TIMER_RESET_DELAY_US_INT, 20);
        devices_[0]->setBoolProperty(OB_PROP_TIMER_RESET_SIGNAL_BOOL, true);
        std::cout << "\nTimestamp reset sent (primary -> all secondaries, delay=20us)" << std::endl;
    }

    // 和 MultiDeviceSync 一致：每 60 秒同步一次
    context_->enableDeviceClockSync(60000);
    std::cout << "Device clock sync enabled (every 60s)" << std::endl;
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
                    std::lock_guard<std::mutex> lock(*mutexes_[camIndex][1]);
                    allFrames_[camIndex][1].push_back(fs);
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