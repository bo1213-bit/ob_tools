# ROS2 Multi-Camera Timestamp Sync Tool — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a self-contained ROS2 package `ros2_sync_tool` that launches N camera nodes, subscribes to image topics, and compares timestamps across devices/streams.

**Architecture:** Launch file starts N `orbbec_camera_node` instances + 1 `sync_analyzer_node`. The analysis node dynamically subscribes to `/camera_N/{depth,color}/image_raw`, collects `FrameStamp` data, then runs `SyncAnalyzerCore` for 3-way comparison and CSV export.

**Tech Stack:** ROS2 (rclcpp), sensor_msgs, ament_cmake, C++17

## Global Constraints

- No modification to any existing `src/` files or top-level `CMakeLists.txt`
- Package is self-contained under `ros2_sync_tool/`
- `FrameStamp` struct duplicated to `ros2_sync_tool/src/frame_stamp.h` (independent copy)
- `sensor_msgs/Image` `header.stamp` used for timestamps (hwTimestampUs = sysTimestampUs for now)
- Launch file drives camera nodes via `serial_number`, `sync_mode` parameters

---

## File Structure

```
ros2_sync_tool/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── multi_camera_sync.launch.py
├── config/
│   └── cameras.yaml
└── src/
    ├── frame_stamp.h               # Copy from ../src/frame_stamp.h
    ├── sync_analyzer_core.h        # Pure math: PairStats, DiffRecord, SyncAnalyzerCore
    ├── sync_analyzer_core.cpp      # Implementation: matchAndDiff, computeStats, printReport, exportCSV
    └── ros2_sync_analyzer_node.cpp # ROS2 node: subscribe, collect, trigger analysis
```

---

### Task 1: Create package skeleton and frame_stamp.h

**Files:**
- Create: `ros2_sync_tool/package.xml`
- Create: `ros2_sync_tool/CMakeLists.txt`
- Create: `ros2_sync_tool/src/frame_stamp.h`

**Interfaces:**
- Produces: `FrameStamp` struct, `StreamType` enum — used by sync_analyzer_core and ros2_sync_analyzer_node

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p ros2_sync_tool/src ros2_sync_tool/launch ros2_sync_tool/config
```

- [ ] **Step 2: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ros2_sync_tool</name>
  <version>0.1.0</version>
  <description>Multi-camera timestamp sync analysis tool using OrbbecSDK ROS2</description>
  <maintainer email="leju@example.com">leju</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>orbbec_camera</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.10)
project(ros2_sync_tool)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- sync_analyzer_core library (pure math, no ROS dependency) ---
add_library(sync_analyzer_core
    src/sync_analyzer_core.cpp
)
target_include_directories(sync_analyzer_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# --- sync_analyzer_node executable ---
add_executable(sync_analyzer_node
    src/ros2_sync_analyzer_node.cpp
)
target_link_libraries(sync_analyzer_node sync_analyzer_core)
ament_target_dependencies(sync_analyzer_node rclcpp sensor_msgs)
target_include_directories(sync_analyzer_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

install(TARGETS sync_analyzer_node sync_analyzer_core
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

- [ ] **Step 4: Write frame_stamp.h (copy from ../src/frame_stamp.h)**

```cpp
// frame_stamp.h
// Shared data structures: StreamType enum + FrameStamp struct
// ROS2 package independent copy — no dependency on libobsensor

#pragma once

#include <cstdint>

enum class StreamType {
    DEPTH = 0,
    COLOR = 1
};

struct FrameStamp {
    int64_t    hwTimestampUs;   // Hardware timestamp (from header.stamp initially)
    int64_t    sysTimestampUs;  // Software/system timestamp (header.stamp)
    int64_t    frameNumber;     // Frame sequence number
    int        deviceIndex;     // Device index (0, 1, 2, ...)
    StreamType streamType;      // DEPTH or COLOR
};
```

- [ ] **Step 5: Commit**

```bash
git add ros2_sync_tool/package.xml ros2_sync_tool/CMakeLists.txt ros2_sync_tool/src/frame_stamp.h
git commit -m "feat: add ros2_sync_tool package skeleton with frame_stamp.h

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 2: Create sync_analyzer_core.h

**Files:**
- Create: `ros2_sync_tool/src/sync_analyzer_core.h`

**Interfaces:**
- Consumes: `FrameStamp`, `StreamType` from frame_stamp.h
- Produces: `SyncAnalyzerCore::PairStats`, `SyncAnalyzerCore::DiffRecord`, `SyncAnalyzerCore` class — used by ros2_sync_analyzer_node.cpp

- [ ] **Step 1: Write sync_analyzer_core.h**

```cpp
// sync_analyzer_core.h
// Pure analysis logic: frame pairing, diff calculation, statistics, CSV export.
// No dependency on libobsensor or ROS2 — only uses FrameStamp and standard library.

#pragma once

#include "frame_stamp.h"

#include <cstdint>
#include <string>
#include <vector>

class SyncAnalyzerCore {
public:
    struct Config {
        int64_t hwThresholdUs = 500;  // Matching threshold for timestamps (us)
    };

    struct PairStats {
        int        deviceI, deviceJ;   // Device indices
        StreamType streamType;         // DEPTH or COLOR (for cross-device comparisons)
        bool       isCrossStream;      // true=same-device cross-stream, false=cross-device same-stream
        int        pairCount;          // Number of successfully paired frames

        // Hardware timestamp diff statistics (us)
        int64_t    hwMinUs, hwMaxUs;
        double     hwMeanUs, hwStddevUs;

        // Global (software) timestamp diff statistics (us)
        int64_t    sysMinUs, sysMaxUs;
        double     sysMeanUs, sysStddevUs;
    };

    struct DiffRecord {
        std::string comparisonType;  // "cross_stream" | "cross_device"
        int         deviceI, deviceJ;
        std::string streamLabel;     // "depth+color" | "depth" | "color"
        int64_t     hwDiffUs;
        int64_t     sysDiffUs;
    };

    // Run analysis on collected frames.
    // frames[deviceIndex][streamTypeIndex][frameIndex]
    void run(const std::vector<std::vector<std::vector<FrameStamp>>>& frames,
             const Config& cfg);

    // Access results
    const std::vector<PairStats>& getCrossStreamStats() const;
    const std::vector<PairStats>& getCrossDeviceDepthStats() const;
    const std::vector<PairStats>& getCrossDeviceColorStats() const;

    // Output
    void printReport() const;
    void exportCSV(const std::string& path) const;

private:
    // Match two FrameStamp sequences by hardware timestamp within threshold.
    // Returns (hwDiffs, sysDiffs) vectors.
    static std::pair<std::vector<int64_t>, std::vector<int64_t>>
    matchAndDiff(const std::vector<FrameStamp>& a,
                 const std::vector<FrameStamp>& b,
                 int64_t hwThresholdUs);

    // Compute statistics from diff vectors.
    static PairStats computeStats(int devI, int devJ,
                                  StreamType st, bool isCrossStream,
                                  const std::vector<int64_t>& hwDiffs,
                                  const std::vector<int64_t>& sysDiffs);

    // Print a single PairStats to stdout.
    static void printOneStats(const PairStats& s);

    // --- Members ---
    std::vector<PairStats> crossStreamStats_;
    std::vector<PairStats> crossDeviceDepthStats_;
    std::vector<PairStats> crossDeviceColorStats_;
    std::vector<DiffRecord>  allDiffs_;
};
```

- [ ] **Step 2: Commit**

```bash
git add ros2_sync_tool/src/sync_analyzer_core.h
git commit -m "feat: add sync_analyzer_core.h — pure analysis logic interface

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 3: Create sync_analyzer_core.cpp

**Files:**
- Create: `ros2_sync_tool/src/sync_analyzer_core.cpp`

**Interfaces:**
- Consumes: `SyncAnalyzerCore`, `PairStats`, `DiffRecord` from sync_analyzer_core.h; `FrameStamp`, `StreamType` from frame_stamp.h
- Produces: Full implementation of matchAndDiff, computeStats, run, printReport, exportCSV

- [ ] **Step 1: Write sync_analyzer_core.cpp**

```cpp
// sync_analyzer_core.cpp
// Pure analysis logic: frame pairing, diff calculation, statistics, CSV export.

#include "sync_analyzer_core.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

// ---------------------------------------------------------------------------
// matchAndDiff — find best-matching frames by hardware timestamp
// ---------------------------------------------------------------------------
std::pair<std::vector<int64_t>, std::vector<int64_t>>
SyncAnalyzerCore::matchAndDiff(
    const std::vector<FrameStamp>& a,
    const std::vector<FrameStamp>& b,
    int64_t hwThresholdUs)
{
    std::vector<int64_t> hwDiffs, sysDiffs;

    if (a.empty() || b.empty()) {
        return {hwDiffs, sysDiffs};
    }

    for (size_t mi = 0; mi < a.size(); mi++) {
        int64_t bestDist = INT64_MAX;
        size_t  bestIdx  = 0;

        for (size_t mj = 0; mj < b.size(); mj++) {
            int64_t dist = std::abs(a[mi].hwTimestampUs - b[mj].hwTimestampUs);
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx  = mj;
            }
        }

        if (bestDist < hwThresholdUs) {
            hwDiffs.push_back(a[mi].hwTimestampUs - b[bestIdx].hwTimestampUs);
            sysDiffs.push_back(a[mi].sysTimestampUs - b[bestIdx].sysTimestampUs);
        }
    }

    return {hwDiffs, sysDiffs};
}

// ---------------------------------------------------------------------------
// computeStats — aggregate diff vectors into PairStats
// ---------------------------------------------------------------------------
SyncAnalyzerCore::PairStats
SyncAnalyzerCore::computeStats(
    int devI, int devJ,
    StreamType st, bool isCrossStream,
    const std::vector<int64_t>& hwDiffs,
    const std::vector<int64_t>& sysDiffs)
{
    PairStats s;
    s.deviceI       = devI;
    s.deviceJ       = devJ;
    s.streamType    = st;
    s.isCrossStream = isCrossStream;
    s.pairCount     = static_cast<int>(hwDiffs.size());

    if (hwDiffs.empty()) return s;

    auto calc = [](const std::vector<int64_t>& diffs,
                   int64_t& minVal, int64_t& maxVal,
                   double& mean, double& stddev) {
        std::vector<int64_t> sorted = diffs;
        std::sort(sorted.begin(), sorted.end());
        minVal = sorted.front();
        maxVal = sorted.back();

        double sum = 0.0;
        for (auto d : diffs) sum += static_cast<double>(d);
        mean = sum / static_cast<double>(diffs.size());

        double sqSum = 0.0;
        for (auto d : diffs) {
            double delta = static_cast<double>(d) - mean;
            sqSum += delta * delta;
        }
        stddev = std::sqrt(sqSum / static_cast<double>(diffs.size()));
    };

    calc(hwDiffs,  s.hwMinUs,  s.hwMaxUs,  s.hwMeanUs,  s.hwStddevUs);
    calc(sysDiffs, s.sysMinUs, s.sysMaxUs, s.sysMeanUs, s.sysStddevUs);

    return s;
}

// ---------------------------------------------------------------------------
// run — three-way comparison
// ---------------------------------------------------------------------------
void SyncAnalyzerCore::run(
    const std::vector<std::vector<std::vector<FrameStamp>>>& frames,
    const Config& cfg)
{
    int deviceCount = static_cast<int>(frames.size());
    const int DEPTH_IDX = static_cast<int>(StreamType::DEPTH);
    const int COLOR_IDX = static_cast<int>(StreamType::COLOR);

    // Clear previous results
    crossStreamStats_.clear();
    crossDeviceDepthStats_.clear();
    crossDeviceColorStats_.clear();
    allDiffs_.clear();

    // 1. Cross-stream: Depth vs Color (same device)
    for (int i = 0; i < deviceCount; i++) {
        if (frames[i][DEPTH_IDX].empty() || frames[i][COLOR_IDX].empty()) continue;

        auto [hwDiffs, sysDiffs] = matchAndDiff(
            frames[i][DEPTH_IDX], frames[i][COLOR_IDX], cfg.hwThresholdUs);

        auto stats = computeStats(i, i, StreamType::DEPTH, true, hwDiffs, sysDiffs);
        crossStreamStats_.push_back(stats);

        for (size_t k = 0; k < hwDiffs.size(); k++) {
            allDiffs_.push_back({"cross_stream", i, i, "depth+color", hwDiffs[k], sysDiffs[k]});
        }
    }

    // 2. Cross-device Depth
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][DEPTH_IDX].empty() || frames[j][DEPTH_IDX].empty()) continue;

            auto [hwDiffs, sysDiffs] = matchAndDiff(
                frames[i][DEPTH_IDX], frames[j][DEPTH_IDX], cfg.hwThresholdUs);

            auto stats = computeStats(i, j, StreamType::DEPTH, false, hwDiffs, sysDiffs);
            crossDeviceDepthStats_.push_back(stats);

            for (size_t k = 0; k < hwDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "depth", hwDiffs[k], sysDiffs[k]});
            }
        }
    }

    // 3. Cross-device Color
    for (int i = 0; i < deviceCount; i++) {
        for (int j = i + 1; j < deviceCount; j++) {
            if (frames[i][COLOR_IDX].empty() || frames[j][COLOR_IDX].empty()) continue;

            auto [hwDiffs, sysDiffs] = matchAndDiff(
                frames[i][COLOR_IDX], frames[j][COLOR_IDX], cfg.hwThresholdUs);

            auto stats = computeStats(i, j, StreamType::COLOR, false, hwDiffs, sysDiffs);
            crossDeviceColorStats_.push_back(stats);

            for (size_t k = 0; k < hwDiffs.size(); k++) {
                allDiffs_.push_back({"cross_device", i, j, "color", hwDiffs[k], sysDiffs[k]});
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------
const std::vector<SyncAnalyzerCore::PairStats>& SyncAnalyzerCore::getCrossStreamStats() const {
    return crossStreamStats_;
}
const std::vector<SyncAnalyzerCore::PairStats>& SyncAnalyzerCore::getCrossDeviceDepthStats() const {
    return crossDeviceDepthStats_;
}
const std::vector<SyncAnalyzerCore::PairStats>& SyncAnalyzerCore::getCrossDeviceColorStats() const {
    return crossDeviceColorStats_;
}

// ---------------------------------------------------------------------------
// printOneStats — single PairStats output
// ---------------------------------------------------------------------------
void SyncAnalyzerCore::printOneStats(const PairStats& s) {
    std::cout << "  Pair count: " << s.pairCount << std::endl;
    std::cout << "  HW Timestamp Diff:" << std::endl;
    std::cout << "    Min=" << s.hwMinUs << "us  Max=" << s.hwMaxUs << "us"
              << "  Mean=" << std::fixed << std::setprecision(1) << s.hwMeanUs
              << "us  Stddev=" << s.hwStddevUs << "us" << std::endl;
    std::cout << "  System Timestamp Diff:" << std::endl;
    std::cout << "    Min=" << s.sysMinUs << "us  Max=" << s.sysMaxUs << "us"
              << "  Mean=" << std::fixed << std::setprecision(1) << s.sysMeanUs
              << "us  Stddev=" << s.sysStddevUs << "us" << std::endl;
}

// ---------------------------------------------------------------------------
// printReport — full console report
// ---------------------------------------------------------------------------
void SyncAnalyzerCore::printReport() const {
    std::cout << "\n==============================================" << std::endl;
    std::cout << "  Timestamp Sync Analysis Report" << std::endl;
    std::cout << "==============================================" << std::endl;

    std::cout << "\n--- 1. Cross-Stream (Depth vs Color) ---" << std::endl;
    if (crossStreamStats_.empty()) {
        std::cout << "  (no data)" << std::endl;
    } else {
        for (const auto& s : crossStreamStats_) {
            std::cout << "Device " << s.deviceI << ":" << std::endl;
            printOneStats(s);
        }
    }

    std::cout << "\n--- 2. Cross-Device Depth ---" << std::endl;
    if (crossDeviceDepthStats_.empty()) {
        std::cout << "  (no data)" << std::endl;
    } else {
        for (const auto& s : crossDeviceDepthStats_) {
            std::cout << "Device " << s.deviceI << " vs Device " << s.deviceJ << ":" << std::endl;
            printOneStats(s);
        }
    }

    std::cout << "\n--- 3. Cross-Device Color ---" << std::endl;
    if (crossDeviceColorStats_.empty()) {
        std::cout << "  (no data)" << std::endl;
    } else {
        for (const auto& s : crossDeviceColorStats_) {
            std::cout << "Device " << s.deviceI << " vs Device " << s.deviceJ << ":" << std::endl;
            printOneStats(s);
        }
    }

    std::cout << "==============================================" << std::endl;
}

// ---------------------------------------------------------------------------
// exportCSV — write raw diffs to CSV
// ---------------------------------------------------------------------------
void SyncAnalyzerCore::exportCSV(const std::string& path) const {
    std::ofstream csvFile(path);
    if (!csvFile.is_open()) {
        std::cerr << "Cannot open CSV: " << path << std::endl;
        return;
    }

    csvFile << "comparison_type,device_i,device_j,stream,hw_diff_us,sys_diff_us" << std::endl;

    for (const auto& d : allDiffs_) {
        csvFile << d.comparisonType << ","
                << d.deviceI << ","
                << d.deviceJ << ","
                << d.streamLabel << ","
                << d.hwDiffUs << ","
                << d.sysDiffUs << std::endl;
    }

    csvFile.close();
    std::cout << "CSV exported: " << path << " (" << allDiffs_.size() << " rows)" << std::endl;
}
```

- [ ] **Step 2: Commit**

```bash
git add ros2_sync_tool/src/sync_analyzer_core.cpp
git commit -m "feat: add sync_analyzer_core.cpp — pure analysis logic implementation

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 4: Create ros2_sync_analyzer_node.cpp

**Files:**
- Create: `ros2_sync_tool/src/ros2_sync_analyzer_node.cpp`

**Interfaces:**
- Consumes: `SyncAnalyzerCore` from sync_analyzer_core.h; `FrameStamp`, `StreamType` from frame_stamp.h; `rclcpp`, `sensor_msgs/msg/image`
- Produces: Standalone ROS2 node executable `sync_analyzer_node`

- [ ] **Step 1: Write ros2_sync_analyzer_node.cpp**

```cpp
// ros2_sync_analyzer_node.cpp
// ROS2 analysis node: subscribes to multi-camera image topics, collects
// frame timestamps, then runs SyncAnalyzerCore for 3-way comparison.

#include "sync_analyzer_core.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class SyncAnalyzerNode : public rclcpp::Node {
public:
    SyncAnalyzerNode() : Node("sync_analyzer_node") {
        // Declare parameters
        this->declare_parameter("camera_names", std::vector<std::string>{});
        this->declare_parameter("stream_types", std::vector<std::string>{"depth", "color"});
        this->declare_parameter("duration_sec", 30);
        this->declare_parameter("hw_threshold_us", 500);
        this->declare_parameter("csv_path", std::string(""));

        // Read parameters
        camera_names_  = this->get_parameter("camera_names").as_string_array();
        stream_types_  = this->get_parameter("stream_types").as_string_array();
        duration_sec_  = this->get_parameter("duration_sec").as_int();
        csv_path_      = this->get_parameter("csv_path").as_string();

        if (camera_names_.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                         "camera_names parameter is empty! No topics to subscribe.");
            rclcpp::shutdown();
            return;
        }

        // Build camera name → index map
        for (size_t i = 0; i < camera_names_.size(); i++) {
            camera_index_map_[camera_names_[i]] = static_cast<int>(i);
        }

        // Build stream type → index map
        for (size_t i = 0; i < stream_types_.size(); i++) {
            stream_index_map_[stream_types_[i]] = static_cast<int>(i);
        }

        // Allocate storage: [camera][stream][frame]
        int camCount   = static_cast<int>(camera_names_.size());
        int streamCount = static_cast<int>(stream_types_.size());
        allFrames_.resize(camCount);
        for (int i = 0; i < camCount; i++) {
            allFrames_[i].resize(streamCount);
        }

        // Create subscriptions
        subscriptions_.reserve(camCount * streamCount);
        for (int ci = 0; ci < camCount; ci++) {
            for (int si = 0; si < streamCount; si++) {
                std::string topic = "/" + camera_names_[ci] + "/"
                                    + stream_types_[si] + "/image_raw";

                auto sub = this->create_subscription<sensor_msgs::msg::Image>(
                    topic, 10,
                    [this, ci, si](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                        this->imageCallback(msg, ci, si);
                    });
                subscriptions_.push_back(sub);

                RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic.c_str());
            }
        }

        // Timer to stop collection after duration_sec
        RCLCPP_INFO(this->get_logger(),
                    "Collecting frames for %d seconds...", duration_sec_);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(duration_sec_),
            [this]() { this->onCollectionDone(); });
    }

private:
    void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg,
                       int cameraIndex, int streamIndex) {
        FrameStamp fs;

        // Extract timestamp from header.stamp
        fs.hwTimestampUs  = static_cast<int64_t>(msg->header.stamp.sec) * 1000000LL
                          + static_cast<int64_t>(msg->header.stamp.nanosec) / 1000LL;
        fs.sysTimestampUs = fs.hwTimestampUs;  // Same source for now
        fs.frameNumber    = 0;                  // Will be index in the vector
        fs.deviceIndex    = cameraIndex;

        // Determine stream type from stream_types_[streamIndex]
        if (stream_types_[streamIndex] == "depth") {
            fs.streamType = StreamType::DEPTH;
        } else if (stream_types_[streamIndex] == "color") {
            fs.streamType = StreamType::COLOR;
        } else {
            fs.streamType = StreamType::DEPTH;  // default
        }

        allFrames_[cameraIndex][streamIndex].push_back(fs);
    }

    void onCollectionDone() {
        RCLCPP_INFO(this->get_logger(), "Collection finished. Running analysis...");

        // Cancel timer and subscriptions
        timer_->cancel();
        for (auto& sub : subscriptions_) {
            sub.reset();
        }
        subscriptions_.clear();

        // Print frame counts
        for (size_t ci = 0; ci < allFrames_.size(); ci++) {
            for (size_t si = 0; si < allFrames_[ci].size(); si++) {
                RCLCPP_INFO(this->get_logger(),
                            "Camera %s / %s: %zu frames",
                            camera_names_[ci].c_str(),
                            stream_types_[si].c_str(),
                            allFrames_[ci][si].size());
            }
        }

        // Run analysis
        SyncAnalyzerCore core;
        SyncAnalyzerCore::Config cfg;
        cfg.hwThresholdUs = this->get_parameter("hw_threshold_us").as_int();
        core.run(allFrames_, cfg);

        // Output
        core.printReport();

        if (!csv_path_.empty()) {
            core.exportCSV(csv_path_);
        }

        RCLCPP_INFO(this->get_logger(), "Done. Shutting down.");
        rclcpp::shutdown();
    }

    // --- Parameters ---
    std::vector<std::string> camera_names_;
    std::vector<std::string> stream_types_;
    int                      duration_sec_;
    std::string              csv_path_;

    // --- Camera/stream name → index maps ---
    std::map<std::string, int> camera_index_map_;
    std::map<std::string, int> stream_index_map_;

    // --- Subscriptions ---
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions_;

    // --- Collected data: [cameraIndex][streamIndex][frameIndex] ---
    std::vector<std::vector<std::vector<FrameStamp>>> allFrames_;

    // --- Timer ---
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncAnalyzerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add ros2_sync_tool/src/ros2_sync_analyzer_node.cpp
git commit -m "feat: add ros2_sync_analyzer_node — ROS2 analysis node

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 5: Create launch file and config YAML

**Files:**
- Create: `ros2_sync_tool/launch/multi_camera_sync.launch.py`
- Create: `ros2_sync_tool/config/cameras.yaml`

**Interfaces:**
- Produces: Launch file starts N `orbbec_camera_node` instances + 1 `sync_analyzer_node`

- [ ] **Step 1: Write cameras.yaml**

```yaml
# Camera configuration for multi_camera_sync.launch.py
# Edit this file to match your hardware setup.

# List of cameras to launch
cameras:
  - name: "camera_0"
    serial_number: "XX001"
    sync_mode: "primary"

  - name: "camera_1"
    serial_number: "XX002"
    sync_mode: "secondary"

# Shared stream configuration
streams:
  depth_width:  848
  depth_height: 480
  depth_fps:    30
  depth_format: "Y16"
  color_width:  848
  color_height: 480
  color_fps:    30
  color_format: "YUYV"

# Analysis node settings
analyzer:
  duration_sec:     30
  hw_threshold_us:  500
  csv_path:          ""    # Set to "result.csv" to export
```

- [ ] **Step 2: Write multi_camera_sync.launch.py**

```python
#!/usr/bin/env python3
"""
Launch file for multi-camera timestamp sync analysis.

Starts N orbbec_camera_node instances (one per camera) plus the
sync_analyzer_node. Edit config/cameras.yaml to match your cameras.

Usage:
  ros2 launch ros2_sync_tool multi_camera_sync.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
import yaml


def generate_launch_description():
    # Find package path
    pkg_dir = get_package_share_directory("ros2_sync_tool")
    config_path = os.path.join(pkg_dir, "config", "cameras.yaml")

    # Load configuration
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    cameras_conf = config.get("cameras", [])
    streams_conf = config.get("streams", {})
    analyzer_conf = config.get("analyzer", {})

    ld = LaunchDescription()

    # Collect camera names for the analyzer node
    camera_names = []

    # Launch one orbbec_camera_node per camera
    for cam in cameras_conf:
        camera_name = cam["name"]
        camera_names.append(camera_name)
        serial = cam.get("serial_number", "")
        sync_mode = cam.get("sync_mode", "standalone")

        # Build camera node parameters
        cam_params = {
            "camera_name": camera_name,
            "serial_number": serial,
            "sync_mode": sync_mode,
            "depth_width": streams_conf.get("depth_width", 848),
            "depth_height": streams_conf.get("depth_height", 480),
            "depth_fps": streams_conf.get("depth_fps", 30),
            "depth_format": streams_conf.get("depth_format", "Y16"),
            "color_width": streams_conf.get("color_width", 848),
            "color_height": streams_conf.get("color_height", 480),
            "color_fps": streams_conf.get("color_fps", 30),
            "color_format": streams_conf.get("color_format", "YUYV"),
            "enable_sync_host_time": True,
            "frames_per_trigger": 1,
        }

        # Primary camera enables trigger output
        if sync_mode == "primary":
            cam_params["trigger_out_enabled"] = True
            cam_params["trigger_out_delay_us"] = 0

        camera_node = Node(
            package="orbbec_camera",
            executable="orbbec_camera_node",
            name=camera_name,
            namespace=camera_name,
            parameters=[cam_params],
            output="screen",
        )
        ld.add_action(camera_node)

        ld.add_action(LogInfo(
            msg=f"Launching camera: {camera_name} (SN={serial}, sync={sync_mode})"
        ))

    # Launch the sync analyzer node
    analyzer_params = {
        "camera_names": camera_names,
        "stream_types": ["depth", "color"],
        "duration_sec": analyzer_conf.get("duration_sec", 30),
        "hw_threshold_us": analyzer_conf.get("hw_threshold_us", 500),
        "csv_path": analyzer_conf.get("csv_path", ""),
    }

    analyzer_node = Node(
        package="ros2_sync_tool",
        executable="sync_analyzer_node",
        name="sync_analyzer_node",
        parameters=[analyzer_params],
        output="screen",
    )
    ld.add_action(analyzer_node)

    ld.add_action(LogInfo(
        msg=f"Launching analyzer with cameras: {camera_names}"
    ))

    return ld
```

- [ ] **Step 3: Commit**

```bash
git add ros2_sync_tool/launch/multi_camera_sync.launch.py ros2_sync_tool/config/cameras.yaml
git commit -m "feat: add launch file and camera config YAML

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Build & Test Instructions

### Build

```bash
cd ob_tools/ros2_sync_tool
colcon build --packages-select ros2_sync_tool
source install/setup.bash
```

### Run

```bash
# Edit config/cameras.yaml with your actual camera serial numbers first

# Launch with default config
ros2 launch ros2_sync_tool multi_camera_sync.launch.py

# Or run analyzer node standalone (cameras must already be running)
ros2 run ros2_sync_tool sync_analyzer_node --ros-args \
  -p camera_names:="['camera_0','camera_1']" \
  -p duration_sec:=30 \
  -p csv_path:="result.csv"
```
