#include "FramePairingManager.hpp"
#include <map>
#include <algorithm>


uint64_t getFrameTimestampMsec(const std::shared_ptr<const ob::Frame> frame) {
    return frame->getTimeStampUs() / 1000;
}

static const char* frameTypeName(OBFrameType t) {
    switch(t) {
        case OB_FRAME_DEPTH:        return "DEPTH";
        case OB_FRAME_COLOR:        return "COLOR";
        case OB_FRAME_IR:           return "IR";
        case OB_FRAME_IR_LEFT:      return "IR_LEFT";
        case OB_FRAME_IR_RIGHT:     return "IR_RIGHT";
        case OB_FRAME_COLOR_LEFT:   return "COLOR_LEFT";
        case OB_FRAME_COLOR_RIGHT:  return "COLOR_RIGHT";
        default:                    return "UNKNOWN";
    }
}


FramePairingManager::FramePairingManager()
    : destroy_(false) {

}

FramePairingManager::~FramePairingManager() {
    release();
}

bool FramePairingManager::pipelineHoldersFrameNotEmpty() {
    if(pipelineHolderList_.size() == 0) {
        return false;
    }

    for(const auto &holder: pipelineHolderList_) {
        if(!holder->isFrameReady()) {
            return false;
        }
    }
    return true;
}

void FramePairingManager::setPipelineHolderList(std::vector<std::shared_ptr<PipelineHolder>> pipelineHolderList) {
	this->pipelineHolderList_ = pipelineHolderList;
    for(auto &&pipelineHolder: pipelineHolderList) {
        int deviceIndex = pipelineHolder->getDeviceIndex();
        if(pipelineHolder->getSensorType() == OB_SENSOR_DEPTH) {
            depthPipelineHolderList_[deviceIndex] = pipelineHolder;
        }
        if(pipelineHolder->getSensorType() == OB_SENSOR_COLOR) {
            colorPipelineHolderList_[deviceIndex] = pipelineHolder;
        }
    }
}

std::vector<std::pair<std::shared_ptr<ob::Frame>, std::shared_ptr<ob::Frame>>> FramePairingManager::getFramePairs() {
    std::vector<std::pair<std::shared_ptr<ob::Frame>, std::shared_ptr<ob::Frame>>> framePairs;
    if(pipelineHolderList_.size() > 0) {
        int depthPipelineHolderSize = static_cast<int>(depthPipelineHolderList_.size());
        auto start = std::chrono::steady_clock::now();
        // Timestamp Matching Mode.
        while(!pipelineHoldersFrameNotEmpty() && !destroy_) {
            // Wait for frames if not yet available (optional: add sleep for simulation)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            if(elapsed > 200) {
                waitTimeouts_++;                 // frame-loss diagnostics
                return framePairs;
            }
        }

        if(destroy_) {
            return framePairs;
        }

        groupsAttempted_++;                      // frame-loss diagnostics

        std::map<int, std::shared_ptr<ob::Frame>> depthFramesMap;
        std::map<int, std::shared_ptr<ob::Frame>> colorFramesMap;

        std::vector<std::shared_ptr<PipelineHolder>> pipelineHolderVector;
        sortFrameMap(pipelineHolderList_, pipelineHolderVector);

        auto        refIter       = pipelineHolderVector.begin();
        const auto &refHolder     = *refIter;
        auto        refTsp        = getFrameTimestampMsec(refHolder->frontFrame());
        auto        refHalfTspGap = refHolder->halfTspGap;

        // ── Phase 1: Peek at ALL frames first, check window ──────────
        //    关键修复：先检查所有帧是否在窗口内，不 pop 任何帧。
        //    如果有帧超窗 → 只丢弃那一帧（pop 它），其余帧保留，
        //    下次 getFramePairs() 用新的队首重新配对。
        int  staleDev   = -1;
        auto staleFrameType = OB_FRAME_DEPTH;  // placeholder
        auto staleGapUs = uint64_t(0);

        for(const auto &item: pipelineHolderVector) {
            auto     tarFrame      = item->frontFrame();
            auto     tarHalfTspGap = item->halfTspGap;
            int      index         = item->getDeviceIndex();
            auto     frameType     = item->getFrameType();
            uint32_t tspHalfGap    = tarHalfTspGap > refHalfTspGap ? tarHalfTspGap : refHalfTspGap;

            auto tarTsp  = getFrameTimestampMsec(tarFrame);
            auto diffTsp = tarTsp - refTsp;
            if(diffTsp > tspHalfGap) {
                staleDev      = index;
                staleFrameType = frameType;
                staleGapUs    = static_cast<uint64_t>(diffTsp) * 1000;
                break;
            }
            refHalfTspGap = tarHalfTspGap;
        }

        if(staleDev >= 0) {
            // 有帧超窗 → 只丢弃这一帧，其余帧原封不动
            // 为什么只丢一帧？因为该相机的后续帧（如 1050ms、1068ms）可能恰好
            // 匹配下一轮的参考帧（当参考帧推进后它们就会落入窗口）。
            // 一次只丢一帧，让参考帧和超窗帧各自逐步靠近，最终对齐。
            for(const auto &item: pipelineHolderVector) {
                if(item->getDeviceIndex() == staleDev && item->getFrameType() == staleFrameType) {
                    item->getFrame();  // pop and discard exactly one stale frame
                    break;
                }
            }
            groupsDiscarded_++;
            discardCount_[staleDev][static_cast<int>(staleFrameType)]++;
            discardGapUs_[staleDev][static_cast<int>(staleFrameType)] += staleGapUs;
            discardMaxGapUs_ = std::max(discardMaxGapUs_, staleGapUs);
            std::cout << "[FRAME-DROP] dev=" << staleDev << " sensor=" << frameTypeName(staleFrameType)
                      << " refTsp=" << refTsp << "ms gap=" << staleGapUs
                      << "us > halfGap=" << refHolder->halfTspGap << "ms"
                      << "  (popped 1 stale frame, others preserved)" << std::endl;
            return framePairs;
        }

        // ── Phase 2: All frames within window → pop all and build group ──
        for(const auto &item: pipelineHolderVector) {
            int  index     = item->getDeviceIndex();
            auto frameType = item->getFrameType();

            if(frameType == OB_FRAME_DEPTH) {
                depthFramesMap[index] = item->getFrame();
            }
            if(frameType == OB_FRAME_COLOR) {
                colorFramesMap[index] = item->getFrame();
            }
        }

        std::cout << "=================================================" << std::endl;

        for(int i = 0; i < depthPipelineHolderSize; i++) {
            auto depthFrame = depthFramesMap[i];
            auto colorFrame = colorFramesMap[i];
            std::cout << "Device#" << i << ", "
                      << " depth(us) "
                      << ", frame timestamp=" << depthFrame->timeStampUs() << ","
                      << "global timestamp = " << depthFrame->globalTimeStampUs() << ","
                      << "system timestamp = " << depthFrame->systemTimeStampUs() << std::endl;

            std::cout << "Device#" << i << ", "
                      << " color(us) "
                      << ", frame timestamp=" << colorFrame->timeStampUs() << ","
                      << "global timestamp = " << colorFrame->globalTimeStampUs() << ","
                      << "system timestamp = " << colorFrame->systemTimeStampUs() << std::endl;

            framePairs.emplace_back(depthFrame, colorFrame);
        }
        groupsSaved_++;                          // frame-loss diagnostics
        return framePairs;
    }

    return framePairs;
}

void FramePairingManager::sortFrameMap(std::vector<std::shared_ptr<PipelineHolder>> &pipelineHolders,
                                       std::vector<std::shared_ptr<PipelineHolder>> &pipelineHolderVector) {
    for(const auto &holder: pipelineHolders) {
        pipelineHolderVector.push_back(holder);
    }

    std::sort(pipelineHolderVector.begin(), pipelineHolderVector.end(), [](const std::shared_ptr<PipelineHolder> &x, const std::shared_ptr<PipelineHolder> &y) {
        auto xTsp = getFrameTimestampMsec(x->frontFrame());
        auto yTsp = getFrameTimestampMsec(y->frontFrame());
        return xTsp < yTsp;
    });
}

void FramePairingManager::release() {
    destroy_ = true;
}

// ============================================================================
//  frame-loss diagnostics
// ============================================================================

void FramePairingManager::resetDiagnostics() {
    groupsAttempted_ = 0;
    groupsSaved_     = 0;
    groupsDiscarded_ = 0;
    waitTimeouts_    = 0;
    discardCount_.clear();
    discardGapUs_.clear();
    discardMaxGapUs_ = 0;
}

void FramePairingManager::printSummary() const {
    std::cout << "\n========== [Frame-Loss Diagnostic] ==========\n";

    // 1) group-level statistics
    std::cout << "Group pairing:\n";
    std::cout << "  groups attempted : " << groupsAttempted_ << "\n";
    std::cout << "  groups saved     : " << groupsSaved_ << "\n";
    std::cout << "  groups discarded : " << groupsDiscarded_ << "\n";
    std::cout << "  wait timeouts    : " << waitTimeouts_
              << "  (getFramePairs returned empty after 200ms without a full frame set)\n";
    if(groupsAttempted_ > 0) {
        std::cout << "  save ratio       : " << (100.0 * groupsSaved_ / groupsAttempted_) << " %\n";
    }

    // 2) per-pipeline frame accounting
    std::cout << "\nPer-pipeline frame accounting (recv / consumed / inQueue / dropped):\n";
    for(const auto &holder : pipelineHolderList_) {
        uint64_t recv    = holder->getFramesReceived();
        uint64_t cons    = holder->getFramesConsumed();
        uint64_t queued  = static_cast<uint64_t>(holder->getFrameQueueSize());
        uint64_t dropped = recv > cons + queued ? (recv - cons - queued) : 0;
        std::cout << "  dev=" << holder->getDeviceIndex()
                  << " sensor=" << frameTypeName(holder->getFrameType())
                  << "  recv=" << recv
                  << "  consumed=" << cons
                  << "  inQueue=" << queued
                  << "  dropped=" << dropped << "\n";
    }

    // 3) discard-cause attribution
    std::cout << "\nDiscard causes (first pipeline whose frame exceeded the halfTspGap window):\n";
    if(discardCount_.empty()) {
        std::cout << "  (none)\n";
    }
    for(const auto &devPair : discardCount_) {
        int dev = devPair.first;
        for(const auto &typePair : devPair.second) {
            int      ft  = typePair.first;
            uint64_t cnt = typePair.second;
            uint64_t gap = 0;
            auto     dit = discardGapUs_.find(dev);
            if(dit != discardGapUs_.end()) {
                auto tit = dit->second.find(ft);
                if(tit != dit->second.end()) {
                    gap = tit->second;
                }
            }
            std::cout << "  dev=" << dev
                      << " sensor=" << frameTypeName(static_cast<OBFrameType>(ft))
                      << "  count=" << cnt
                      << "  avgGap=" << (cnt ? gap / cnt : 0) << "us"
                      << "  maxGap(all)=" << discardMaxGapUs_ << "us\n";
        }
    }
    std::cout << "==============================================\n";
}
