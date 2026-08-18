#pragma once
#include <libobsensor/ObSensor.hpp>
#include <iostream>
#include <vector>
#include <mutex>
#include <queue>
#include <condition_variable>


class PipelineHolder {
public:
    PipelineHolder(std::shared_ptr<ob::Pipeline> pipeline, OBSensorType sensorType, std::string deviceSN, int deviceIndex);
    ~PipelineHolder();


public:
    void startStream();

    void processFrame(std::shared_ptr<ob::FrameSet> frameSet);

    bool isFrameReady();

    std::shared_ptr<ob::Frame> frontFrame();

    void popFrame();

    std::shared_ptr<ob::Frame> getFrame();

    // 非阻塞取帧：队列有帧则弹出并返回，没有则立即返回 nullptr（用于主循环排空队列）
    std::shared_ptr<ob::Frame> tryGetFrame();

    void stopStream();

    void release();

    void handleStreamError(const ob::Error &e);

    OBFrameType mapFrameType(OBSensorType sensorType);

    std::string getSerialNumber() {
        return deviceSN_;
    }

    OBSensorType getSensorType() {
        return sensorType_;
    }

    OBFrameType getFrameType() {
        return frameType_;
    }

    int getDeviceIndex(){
        return deviceIndex_;
    }

   int getFrameQueueSize() {
	       std::lock_guard<std::mutex> lock(queueMutex_);
	        return static_cast<int>(obFrames.size());
    }

    // ---- frame-loss diagnostics ----
    uint64_t getFramesReceived() const { return framesReceived_; }
    uint64_t getFramesConsumed() const { return framesConsumed_; }

private:
    bool startStream_;

    std::shared_ptr<ob::Pipeline> pipeline_;
    OBSensorType                  sensorType_;
    OBFrameType                   frameType_;
    std::string                   deviceSN_;
    int                           deviceIndex_;

    std::condition_variable condVar_;
    std::mutex              queueMutex_;
    uint32_t                maxFrameSize_ = 16;

    std::queue<std::shared_ptr<ob::Frame>> obFrames;

    uint64_t framesReceived_ = 0;   // frames pushed into the queue (produced by sensor)
    uint64_t framesConsumed_ = 0;   // frames popped by the pairing (saved into a group)

public:
    uint32_t halfTspGap;
};
