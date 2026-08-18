#pragma once
#include "PipelineHolder.hpp"
#include <libobsensor/ObSensor.hpp>
#include <iostream>
#include <vector>
#include <mutex>
#include <condition_variable>


class FramePairingManager {
public:
    FramePairingManager();
    ~FramePairingManager();

private:
    bool pipelineHoldersFrameNotEmpty();
    void sortFrameMap(std::vector<std::shared_ptr<PipelineHolder>> &pipelineHolders, std::vector<std::shared_ptr<PipelineHolder>> &pipelineHolderVector);

public:
    void setPipelineHolderList(std::vector<std::shared_ptr<PipelineHolder>> pipelineHolderList);

    std::vector<std::pair<std::shared_ptr<ob::Frame>, std::shared_ptr<ob::Frame>>> getFramePairs();

    void release();

    // ---- frame-loss diagnostics ----
    void printSummary() const;
    void resetDiagnostics();

private:
    bool     destroy_;
    bool     timestampPairingEnable_;
    uint64_t timestampPairingRange_;

    std::vector<std::shared_ptr<PipelineHolder>> pipelineHolderList_;
    std::map<int, std::shared_ptr<PipelineHolder>> depthPipelineHolderList_;
    std::map<int, std::shared_ptr<PipelineHolder>> colorPipelineHolderList_;

    // ---- frame-loss diagnostic counters ----
    uint64_t groupsAttempted_ = 0;    // times getFramePairs() tried to form a group
    uint64_t groupsSaved_     = 0;    // times a full group was successfully returned
    uint64_t groupsDiscarded_ = 0;    // times a group was discarded (frame out of window)
    uint64_t waitTimeouts_    = 0;    // times the 200ms "wait for all queues" timed out
    // discard attribution: [deviceIndex][frameType] -> count / summed gap (us)
    std::map<int, std::map<int, uint64_t>> discardCount_;
    std::map<int, std::map<int, uint64_t>> discardGapUs_;
    uint64_t discardMaxGapUs_ = 0;
};
