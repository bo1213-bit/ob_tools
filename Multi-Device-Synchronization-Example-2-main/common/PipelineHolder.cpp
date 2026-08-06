#include "PipelineHolder.hpp"


PipelineHolder::PipelineHolder(std::shared_ptr<ob::Pipeline> pipeline, OBSensorType sensorType, std::string deviceSN, int deviceIndex)
    : startStream_(false), pipeline_(pipeline), sensorType_(sensorType), deviceSN_(deviceSN), deviceIndex_(deviceIndex) {
}

PipelineHolder::~PipelineHolder() {
    release();
}

void PipelineHolder::startStream() {
    std::cout << "startStream: " << deviceSN_ << " sensorType:" << sensorType_ << std::endl;
    try {
        if(pipeline_) {
            auto profileList = pipeline_->getStreamProfileList(sensorType_);

            // 方案 B：遍历 profile 列表，优先挑 1280x800 @ 30fps（和 Depth 一致，便于同步）。
            // 30fps 不存在则退回 1280x800（任意帧率），再退回第一个可用。
            // 不依赖 OrbbecSDKConfig.xml —— getProfile(OB_PROFILE_DEFAULT) 取列表第 0 个，由固件决定。
            std::shared_ptr<ob::VideoStreamProfile> streamProfile;
            std::shared_ptr<ob::VideoStreamProfile> any1280x800;   // 兜底：任意帧率的 1280x800
            for(uint32_t i = 0; i < profileList->getCount(); i++) {
                auto prof = profileList->getProfile(i)->as<ob::VideoStreamProfile>();
                if(!prof) continue;
                if(prof->getWidth() == 1280 && prof->getHeight() == 800) {
                    if(!any1280x800) any1280x800 = prof;
                    if(prof->getFps() == 30) {
                        streamProfile = prof;
                        break;
                    }
                }
            }
            if(!streamProfile) streamProfile = any1280x800;   // 无 30fps 的 1280x800 → 用任意帧率的 1280x800

            if(!streamProfile) {
                // 没找到 1280x800 —— 打印所有可用 profile，方便排查相机到底支持哪些
                std::cerr << "  !! no 1280x800 profile for " << deviceSN_ << ", available profiles:" << std::endl;
                for(uint32_t i = 0; i < profileList->getCount(); i++) {
                    auto prof = profileList->getProfile(i)->as<ob::VideoStreamProfile>();
                    if(prof) {
                        std::cerr << "    " << prof->getWidth() << "x" << prof->getHeight()
                                  << " @ " << prof->getFps() << "fps fmt=" << prof->getFormat() << std::endl;
                    }
                }
                // 回退到第一个可用的
                streamProfile = profileList->getProfile(0)->as<ob::VideoStreamProfile>();
            }

            frameType_ = mapFrameType(sensorType_);

            auto fps    = streamProfile->getFps();
            halfTspGap = static_cast<uint32_t>(500.0f / fps + 0.5);

            // ★ 打印实际启用的分辨率/帧率/格式（用于确认是否 1280x800 生效）
            const char *sensorName = (sensorType_ == OB_SENSOR_DEPTH) ? "DEPTH"
                                   : (sensorType_ == OB_SENSOR_COLOR) ? "COLOR"
                                                                      : "OTHER";
            std::cout << "  [" << deviceSN_ << "] " << sensorName
                      << " enabled profile: " << streamProfile->getWidth() << "x"
                      << streamProfile->getHeight() << " @ " << fps << "fps"
                      << " fmt=" << streamProfile->getFormat() << std::endl;

            std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
            config->enableStream(streamProfile);

            pipeline_->start(config, [this](std::shared_ptr<ob::FrameSet> frameSet) {
                processFrame(frameSet);
            });
            startStream_ = true;
        }
    }
    catch(ob::Error &e) {
        std::cerr << "starting stream failed: " << deviceSN_ << std::endl;
        handleStreamError(e);
    }
}

void PipelineHolder::processFrame(std::shared_ptr<ob::FrameSet> frameSet) {
    if(!frameSet) {
        std::cerr << "Invalid frameSet received." << std::endl;
        return;
    }

    if(!startStream_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        auto obFrame = frameSet->getFrame(frameType_);
        if(obFrame) {
            framesReceived_++;                       // frame-loss diagnostics
            if(obFrames.size() >= static_cast<size_t>(maxFrameSize_)) {
                obFrames.pop();
            }
            obFrames.push(obFrame);
        }
    }

    condVar_.notify_all();
}

bool PipelineHolder::isFrameReady() {
    {
        std::unique_lock<std::mutex> lock(queueMutex_);
        condVar_.wait(lock, [this]() { return !obFrames.empty() || startStream_; });
        if(startStream_ && obFrames.empty()) {
            return false;
        }
    }
    return true;
}

std::shared_ptr<ob::Frame> PipelineHolder::frontFrame() {
    {
        std::unique_lock<std::mutex> lock(queueMutex_);
        condVar_.wait(lock, [this]() { return !obFrames.empty() || startStream_; });
        if(startStream_ && obFrames.empty()) {
            return nullptr;
        }
        auto frame = obFrames.front();
        return frame;
    }
}

void PipelineHolder::popFrame() {
    {
        std::unique_lock<std::mutex> lock(queueMutex_);
        condVar_.wait(lock, [this]() { return !obFrames.empty() || startStream_; });
        if(startStream_ && obFrames.empty()) {
            return;
        }
        obFrames.pop();
    }
}

std::shared_ptr<ob::Frame> PipelineHolder::getFrame() {
    {
        std::unique_lock<std::mutex> lock(queueMutex_);
        condVar_.wait(lock, [this]() { return !obFrames.empty() || startStream_; });
        if(startStream_ && obFrames.empty()) {
            return nullptr;
        }
        auto frame = obFrames.front();
        obFrames.pop();
        framesConsumed_++;                   // frame-loss diagnostics
        return frame;
    }
}

std::shared_ptr<ob::Frame> PipelineHolder::tryGetFrame() {
    std::lock_guard<std::mutex> lock(queueMutex_);
    if(obFrames.empty()) {
        return nullptr;
    }
    auto frame = obFrames.front();
    obFrames.pop();
    framesConsumed_++;
    return frame;
}

void PipelineHolder::stopStream() {
    try {
        if(pipeline_) {
            std::cout << "stopStream: " << deviceSN_ << " sensorType:" << sensorType_ << std::endl;
            startStream_ = false;
            pipeline_->stop();
        }
    }
    catch(ob::Error &e) {
        std::cerr << "stopping stream failed: " << deviceSN_ << std::endl;
        std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\nstatus:" << e.getStatus()
                  << "\ntype:" << e.getExceptionType() << std::endl;
    }
}

void PipelineHolder::release() {
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        startStream_ = false;
    }
    condVar_.notify_all();
}

void PipelineHolder::handleStreamError(const ob::Error &e) {
    std::cerr << "Function: " << e.getName() << "\nArgs: " << e.getArgs() << "\nMessage: " << e.getMessage() << "\nstatus:" << e.getStatus()
              << "\nType: " << e.getExceptionType() << std::endl;
}

OBFrameType PipelineHolder::mapFrameType(OBSensorType sensorType) {
    switch(sensorType) {
    case OB_SENSOR_COLOR:
        return OB_FRAME_COLOR;
    case OB_SENSOR_COLOR_LEFT:
        return OB_FRAME_COLOR_LEFT;
    case OB_SENSOR_COLOR_RIGHT:
        return OB_FRAME_COLOR_RIGHT;
    case OB_SENSOR_IR:
        return OB_FRAME_IR;
    case OB_SENSOR_IR_LEFT:
        return OB_FRAME_IR_LEFT;
    case OB_SENSOR_IR_RIGHT:
        return OB_FRAME_IR_RIGHT;
    case OB_SENSOR_DEPTH:
        return OB_FRAME_DEPTH;
    default:
        return OBFrameType::OB_FRAME_UNKNOWN;
    }
}
