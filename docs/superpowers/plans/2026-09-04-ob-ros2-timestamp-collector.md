# OB ROS2 Timestamp Collector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone ROS 2 C++ package under `OB ROS2/` that subscribes to configured image topics, records timestamp and image metadata, optionally saves raw image bytes, and writes a per-capture CSV.

**Architecture:** `timestamp_collector_node` accepts an explicit list of `(camera_name, stream_type, topic)` subscriptions from ROS parameters. Each callback records the incoming image's `header.stamp`, node receive time, metadata, and per-stream index in `timestamps.csv`; an optional bounded writer queue saves raw image bytes asynchronously. The package does not start Orbbec cameras, match frames, calculate synchronization quality, or control hardware triggers.

**Tech Stack:** ROS 2 Humble-compatible C++17, `ament_cmake`, `rclcpp`, `sensor_msgs`, launch Python, standard C++ filesystem/threading/CSV I/O.

**Spec:** `docs/superpowers/specs/2026-09-04-ob-ros2-timestamp-collector-design.md`

## Global Constraints

- Create all new package files only under `OB ROS2/`; do not modify existing `src/` or the top-level `CMakeLists.txt`.
- Package name is exactly `ob_ros2_timestamp_collector`.
- The package only subscribes to standard `sensor_msgs/msg/Image` topics; it does not start or configure `orbbec_camera` nodes.
- `header.stamp` is stored as `header_stamp_us`; it becomes a cross-camera timestamp candidate only when the driver is configured with `time_domain:=global` and `enable_sync_host_time:=false`.
- Do not implement timestamp matching, synchronization quality statistics, or any GPIO/PWM/hardware-trigger control.
- Raw images are saved as `.bin` files with original byte layout, never converted to PNG.
- Image writing must occur off the ROS subscription callback path using a bounded queue; queue overflow must retain the CSV timestamp record and report dropped image-save jobs.
- The node must fail before subscribing if required configuration or output initialization is invalid.

---

## File Structure

```text
OB ROS2/
├── CMakeLists.txt                              # ament build definition and test targets
├── package.xml                                 # ROS 2 package metadata and dependencies
├── README.md                                   # build, configuration, Orbbec, and output usage
├── config/
│   └── collector.yaml                          # two-camera example parameters
├── launch/
│   └── timestamp_collector.launch.py           # starts only collector node with YAML parameters
├── include/ob_ros2_timestamp_collector/
│   ├── capture_record.hpp                      # immutable CSV record and CSV helper declarations
│   └── collector_core.hpp                      # non-ROS capture/session and bounded writer queue logic
├── src/
│   ├── collector_core.cpp                      # CSV/session setup, writer thread, raw file writing
│   └── timestamp_collector_node.cpp            # ROS parameter parsing and Image subscriptions
└── test/
    ├── test_collector_core.cpp                 # gtest for CSV, naming, limits, and image writing
    └── test_capture_record.cpp                 # gtest for timestamp conversion and CSV escaping
```

The public `collector_core` layer owns output paths, thread-safe CSV writes, frame limits, and optional raw-image save jobs. The ROS node owns only ROS parameter validation, QoS selection, subscriptions, timestamp conversion, and lifecycle timer. This keeps output behavior testable without ROS graph fixtures.

---

### Task 1: Create package metadata and testable record model

**Files:**
- Create: `OB ROS2/package.xml`
- Create: `OB ROS2/CMakeLists.txt`
- Create: `OB ROS2/include/ob_ros2_timestamp_collector/capture_record.hpp`
- Create: `OB ROS2/test/test_capture_record.cpp`

**Interfaces:**
- Produces `ob_ros2_timestamp_collector::CaptureRecord` with fields `camera_name`, `stream_type`, `frame_index`, `header_stamp_us`, `receive_stamp_us`, `frame_id`, `width`, `height`, `encoding`, `step`, `data_size`, and `image_path`.
- Produces `int64_t stamp_to_microseconds(int32_t sec, uint32_t nanosec)` and `std::string csv_escape(const std::string & value)`.
- Produces an ament CMake package with gtest support.

- [ ] **Step 1: Create the package and public include directories**

Run:

```powershell
New-Item -ItemType Directory -Force "OB ROS2/include/ob_ros2_timestamp_collector", "OB ROS2/src", "OB ROS2/test", "OB ROS2/config", "OB ROS2/launch"
```

Expected: the five directories exist under `OB ROS2/`.

- [ ] **Step 2: Write the failing record-model tests**

Create `OB ROS2/test/test_capture_record.cpp`:

```cpp
#include "ob_ros2_timestamp_collector/capture_record.hpp"

#include <gtest/gtest.h>

namespace collector = ob_ros2_timestamp_collector;

TEST(CaptureRecord, ConvertsRosStampToMicroseconds)
{
  EXPECT_EQ(collector::stamp_to_microseconds(12, 345678999U), 12345678);
  EXPECT_EQ(collector::stamp_to_microseconds(0, 999U), 0);
}

TEST(CaptureRecord, EscapesCsvQuotesAndCommas)
{
  EXPECT_EQ(collector::csv_escape("camera_01"), "camera_01");
  EXPECT_EQ(collector::csv_escape("camera,01"), "\"camera,01\"");
  EXPECT_EQ(collector::csv_escape("a\"b"), "\"a\"\"b\"");
}
```

- [ ] **Step 3: Add the first build definition and run the test to verify it fails**

Create `OB ROS2/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(ob_ros2_timestamp_collector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_capture_record test/test_capture_record.cpp)
  target_include_directories(test_capture_record PRIVATE include)
endif()

ament_package()
```

Create `OB ROS2/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ob_ros2_timestamp_collector</name>
  <version>0.1.0</version>
  <description>Subscribe to ROS 2 image topics and capture timestamp metadata with optional raw image bytes.</description>
  <maintainer email="maintainer@example.com">ob_tools maintainer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Run from the ROS 2 workspace that contains `OB ROS2/`:

```bash
colcon test --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
```

Expected: build/test failure because `capture_record.hpp` does not exist.

- [ ] **Step 4: Implement timestamp and CSV helpers**

Create `OB ROS2/include/ob_ros2_timestamp_collector/capture_record.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <string>

namespace ob_ros2_timestamp_collector
{

struct CaptureRecord
{
  std::string camera_name;
  std::string stream_type;
  uint64_t frame_index{};
  int64_t header_stamp_us{};
  int64_t receive_stamp_us{};
  std::string frame_id;
  uint32_t width{};
  uint32_t height{};
  std::string encoding;
  uint32_t step{};
  size_t data_size{};
  std::string image_path;
};

inline int64_t stamp_to_microseconds(const int32_t sec, const uint32_t nanosec)
{
  return static_cast<int64_t>(sec) * 1000000LL + static_cast<int64_t>(nanosec) / 1000LL;
}

inline std::string csv_escape(const std::string & value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }

  std::string escaped{"\""};
  for (const char character : value) {
    if (character == '\"') {
      escaped += "\"\"";
    } else {
      escaped += character;
    }
  }
  escaped += '\"';
  return escaped;
}

}  // namespace ob_ros2_timestamp_collector
```

- [ ] **Step 5: Run the record-model test to verify it passes**

Run:

```bash
colcon test --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
colcon test-result --verbose
```

Expected: `test_capture_record` passes.

- [ ] **Step 6: Commit the package foundation**

```bash
git add "OB ROS2/package.xml" "OB ROS2/CMakeLists.txt" "OB ROS2/include/ob_ros2_timestamp_collector/capture_record.hpp" "OB ROS2/test/test_capture_record.cpp"
git commit -m "feat: add ROS2 timestamp collector package foundation"
```

---

### Task 2: Implement the output session and bounded asynchronous raw-image writer

**Files:**
- Create: `OB ROS2/include/ob_ros2_timestamp_collector/collector_core.hpp`
- Create: `OB ROS2/src/collector_core.cpp`
- Create: `OB ROS2/test/test_collector_core.cpp`
- Modify: `OB ROS2/CMakeLists.txt`

**Interfaces:**
- Consumes `CaptureRecord` from `capture_record.hpp`.
- Produces `CollectorCore::Config { std::filesystem::path output_dir; bool save_images; size_t writer_queue_capacity; size_t flush_every_n_records; }`.
- Produces `CollectorCore::start()`, `CollectorCore::record(CaptureRecord record, std::vector<uint8_t> image_bytes)`, `CollectorCore::stop()`, `CollectorCore::capture_directory() const`, and `CollectorCore::dropped_image_jobs() const`.
- `record()` writes a CSV row synchronously and enqueues raw image bytes only when `save_images` is true; it returns `false` after `stop()`.

- [ ] **Step 1: Write failing output-session tests**

Create `OB ROS2/test/test_collector_core.cpp`:

```cpp
#include "ob_ros2_timestamp_collector/collector_core.hpp"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

namespace collector = ob_ros2_timestamp_collector;

class CollectorCoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = std::filesystem::temp_directory_path() / "ob_ros2_collector_test";
    std::filesystem::remove_all(root_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(root_);
  }

  std::filesystem::path root_;
};

TEST_F(CollectorCoreTest, CreatesCsvWithHeaderAndRecord)
{
  collector::CollectorCore core({root_, false, 2, 1});
  core.start();

  collector::CaptureRecord record{"camera_01", "color", 0, 100, 200, "camera_color", 640, 480,
    "rgb8", 1920, 921600, ""};
  EXPECT_TRUE(core.record(record, {}));
  core.stop();

  std::ifstream csv(core.capture_directory() / "timestamps.csv");
  std::string header;
  std::string line;
  std::getline(csv, header);
  std::getline(csv, line);
  EXPECT_EQ(header, "camera_name,stream_type,frame_index,header_stamp_us,receive_stamp_us,frame_id,width,height,encoding,step,data_size,image_path");
  EXPECT_NE(line.find("camera_01,color,0,100,200"), std::string::npos);
}

TEST_F(CollectorCoreTest, SavesOriginalBytesAndWritesRelativePath)
{
  collector::CollectorCore core({root_, true, 2, 1});
  core.start();

  collector::CaptureRecord record{"camera_01", "depth", 3, 100, 200, "camera_depth", 2, 2,
    "16UC1", 4, 8, ""};
  EXPECT_TRUE(core.record(record, {1, 2, 3, 4, 5, 6, 7, 8}));
  core.stop();

  const auto image_path = core.capture_directory() / "images/camera_01/depth/00000003.bin";
  EXPECT_TRUE(std::filesystem::exists(image_path));
  EXPECT_EQ(std::filesystem::file_size(image_path), 8U);
}
```

- [ ] **Step 2: Register the test and run it to verify it fails**

Append to `OB ROS2/CMakeLists.txt` inside `if(BUILD_TESTING)`:

```cmake
ament_add_gtest(test_collector_core test/test_collector_core.cpp)
target_include_directories(test_collector_core PRIVATE include)
```

Run:

```bash
colcon test --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
```

Expected: failure because `collector_core.hpp` does not exist.

- [ ] **Step 3: Define the core API**

Create `OB ROS2/include/ob_ros2_timestamp_collector/collector_core.hpp`:

```cpp
#pragma once

#include "ob_ros2_timestamp_collector/capture_record.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace ob_ros2_timestamp_collector
{

class CollectorCore
{
public:
  struct Config
  {
    std::filesystem::path output_dir;
    bool save_images{false};
    size_t writer_queue_capacity{64};
    size_t flush_every_n_records{1};
  };

  explicit CollectorCore(Config config);
  ~CollectorCore();

  void start();
  bool record(CaptureRecord record, std::vector<uint8_t> image_bytes);
  void stop();

  std::filesystem::path capture_directory() const;
  uint64_t dropped_image_jobs() const;

private:
  struct ImageJob
  {
    std::filesystem::path path;
    std::vector<uint8_t> bytes;
  };

  std::filesystem::path image_relative_path(const CaptureRecord & record) const;
  void writer_loop();
  void write_csv_record(const CaptureRecord & record);

  Config config_;
  std::filesystem::path capture_directory_;
  std::ofstream csv_file_;
  mutable std::mutex csv_mutex_;
  size_t records_since_flush_{0};

  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::queue<ImageJob> image_jobs_;
  std::thread writer_thread_;
  bool writer_running_{false};
  bool accepting_records_{false};
  std::atomic<uint64_t> dropped_image_jobs_{0};
};

}  // namespace ob_ros2_timestamp_collector
```

- [ ] **Step 4: Implement capture-directory creation, CSV writing, and writer loop**

Create `OB ROS2/src/collector_core.cpp` with these required behaviors:

```cpp
#include "ob_ros2_timestamp_collector/collector_core.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ob_ros2_timestamp_collector
{
namespace
{
std::string capture_directory_name()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &time);
#else
  localtime_r(&time, &local_time);
#endif
  std::ostringstream result;
  result << "capture_" << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return result.str();
}
}  // namespace

CollectorCore::CollectorCore(Config config) : config_(std::move(config)) {}
CollectorCore::~CollectorCore() { stop(); }

void CollectorCore::start()
{
  if (accepting_records_) {
    throw std::logic_error("CollectorCore has already started");
  }
  if (config_.output_dir.empty()) {
    throw std::invalid_argument("output_dir must not be empty");
  }

  capture_directory_ = config_.output_dir / capture_directory_name();
  std::filesystem::create_directories(capture_directory_);
  csv_file_.open(capture_directory_ / "timestamps.csv", std::ios::out | std::ios::trunc);
  if (!csv_file_.is_open()) {
    throw std::runtime_error("cannot open timestamps.csv");
  }
  csv_file_ << "camera_name,stream_type,frame_index,header_stamp_us,receive_stamp_us,frame_id,width,height,encoding,step,data_size,image_path\n";
  csv_file_.flush();

  accepting_records_ = true;
  if (config_.save_images) {
    writer_running_ = true;
    writer_thread_ = std::thread(&CollectorCore::writer_loop, this);
  }
}

bool CollectorCore::record(CaptureRecord record, std::vector<uint8_t> image_bytes)
{
  if (!accepting_records_) {
    return false;
  }

  if (config_.save_images) {
    const auto relative_path = image_relative_path(record);
    record.image_path = relative_path.generic_string();
    ImageJob job{capture_directory_ / relative_path, std::move(image_bytes)};
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (image_jobs_.size() < config_.writer_queue_capacity) {
        image_jobs_.push(std::move(job));
        queue_condition_.notify_one();
      } else {
        record.image_path.clear();
        ++dropped_image_jobs_;
      }
    }
  }

  write_csv_record(record);
  return true;
}

void CollectorCore::stop()
{
  accepting_records_ = false;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    writer_running_ = false;
  }
  queue_condition_.notify_all();
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
  std::lock_guard<std::mutex> lock(csv_mutex_);
  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }
}

std::filesystem::path CollectorCore::capture_directory() const { return capture_directory_; }
uint64_t CollectorCore::dropped_image_jobs() const { return dropped_image_jobs_.load(); }

std::filesystem::path CollectorCore::image_relative_path(const CaptureRecord & record) const
{
  std::ostringstream filename;
  filename << std::setw(8) << std::setfill('0') << record.frame_index << ".bin";
  return std::filesystem::path{"images"} / record.camera_name / record.stream_type / filename.str();
}

void CollectorCore::write_csv_record(const CaptureRecord & record)
{
  std::lock_guard<std::mutex> lock(csv_mutex_);
  csv_file_ << csv_escape(record.camera_name) << ',' << csv_escape(record.stream_type) << ','
            << record.frame_index << ',' << record.header_stamp_us << ',' << record.receive_stamp_us << ','
            << csv_escape(record.frame_id) << ',' << record.width << ',' << record.height << ','
            << csv_escape(record.encoding) << ',' << record.step << ',' << record.data_size << ','
            << csv_escape(record.image_path) << '\n';
  ++records_since_flush_;
  if (records_since_flush_ >= config_.flush_every_n_records) {
    csv_file_.flush();
    records_since_flush_ = 0;
  }
}

void CollectorCore::writer_loop()
{
  while (true) {
    ImageJob job;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_condition_.wait(lock, [this]() { return !image_jobs_.empty() || !writer_running_; });
      if (image_jobs_.empty() && !writer_running_) {
        return;
      }
      job = std::move(image_jobs_.front());
      image_jobs_.pop();
    }
    std::filesystem::create_directories(job.path.parent_path());
    std::ofstream image_file(job.path, std::ios::binary | std::ios::trunc);
    if (image_file.is_open()) {
      image_file.write(reinterpret_cast<const char *>(job.bytes.data()), static_cast<std::streamsize>(job.bytes.size()));
    }
  }
}

}  // namespace ob_ros2_timestamp_collector
```

- [ ] **Step 5: Add the core library and test linkage**

Replace the target section in `OB ROS2/CMakeLists.txt` with:

```cmake
add_library(collector_core src/collector_core.cpp)
target_include_directories(collector_core PUBLIC include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_capture_record test/test_capture_record.cpp)
  target_include_directories(test_capture_record PRIVATE include)

  ament_add_gtest(test_collector_core test/test_collector_core.cpp)
  target_link_libraries(test_collector_core collector_core)
  target_include_directories(test_collector_core PRIVATE include)
endif()
```

- [ ] **Step 6: Run tests to verify CSV and raw image behavior**

Run:

```bash
colcon test --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
colcon test-result --verbose
```

Expected: `test_capture_record` and `test_collector_core` pass.

- [ ] **Step 7: Commit the testable output core**

```bash
git add "OB ROS2/include/ob_ros2_timestamp_collector/collector_core.hpp" "OB ROS2/src/collector_core.cpp" "OB ROS2/test/test_collector_core.cpp" "OB ROS2/CMakeLists.txt"
git commit -m "feat: add timestamp collector output core"
```

---

### Task 3: Implement parameterized ROS Image subscriptions and collection lifecycle

**Files:**
- Create: `OB ROS2/src/timestamp_collector_node.cpp`
- Modify: `OB ROS2/CMakeLists.txt`

**Interfaces:**
- Consumes `CollectorCore` and `CaptureRecord`.
- Produces executable `timestamp_collector_node`.
- Declares parameters `subscriptions` (`string[]`), `output_dir` (`string`), `duration_sec` (`int`), `save_images` (`bool`), `max_frames_per_stream` (`int`), `qos_depth` (`int`), `use_sensor_data_qos` (`bool`), `writer_queue_capacity` (`int`), and `flush_every_n_records` (`int`).
- Every `subscriptions` item has exact format `camera_name|stream_type|topic`; only stream types `color` and `depth` are accepted.

- [ ] **Step 1: Write the node parameter parser with strict validation**

Create `OB ROS2/src/timestamp_collector_node.cpp` and define:

```cpp
struct SubscriptionSpec
{
  std::string camera_name;
  std::string stream_type;
  std::string topic;
  uint64_t frame_index{0};
};

SubscriptionSpec parse_subscription_spec(const std::string & value)
{
  const auto first = value.find('|');
  const auto second = first == std::string::npos ? std::string::npos : value.find('|', first + 1);
  if (first == std::string::npos || second == std::string::npos || value.find('|', second + 1) != std::string::npos) {
    throw std::invalid_argument("subscription must use camera_name|stream_type|topic: " + value);
  }
  SubscriptionSpec result{value.substr(0, first), value.substr(first + 1, second - first - 1), value.substr(second + 1)};
  if (result.camera_name.empty() || result.topic.empty()) {
    throw std::invalid_argument("camera_name and topic must not be empty: " + value);
  }
  if (result.stream_type != "color" && result.stream_type != "depth") {
    throw std::invalid_argument("stream_type must be color or depth: " + result.stream_type);
  }
  return result;
}
```

- [ ] **Step 2: Implement `TimestampCollectorNode` startup and core initialization**

In the same file, create a node class that declares and reads all required parameters. Validate:

```cpp
if (subscriptions_parameter.empty()) {
  throw std::invalid_argument("subscriptions must contain at least one camera_name|stream_type|topic item");
}
if (output_dir.empty()) {
  throw std::invalid_argument("output_dir must not be empty");
}
if (duration_sec < 0 || max_frames_per_stream < 0 || qos_depth <= 0 || writer_queue_capacity <= 0 || flush_every_n_records <= 0) {
  throw std::invalid_argument("duration, frame limit, QoS depth, queue capacity, and flush count must be valid non-negative/positive values");
}
```

Build a unique key as `camera_name + "|" + stream_type`; reject duplicate keys. Construct `CollectorCore` with the configured output values and invoke `core_->start()` before creating any subscriptions. Catch exceptions in `main`, log with `RCLCPP_FATAL`, call `rclcpp::shutdown()`, and return nonzero.

- [ ] **Step 3: Implement QoS and image callback**

Use this callback logic for each `SubscriptionSpec`:

```cpp
void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, const size_t spec_index)
{
  auto & spec = subscriptions_.at(spec_index);
  if (max_frames_per_stream_ > 0 && spec.frame_index >= static_cast<uint64_t>(max_frames_per_stream_)) {
    return;
  }

  CaptureRecord record;
  record.camera_name = spec.camera_name;
  record.stream_type = spec.stream_type;
  record.frame_index = spec.frame_index++;
  record.header_stamp_us = stamp_to_microseconds(msg->header.stamp.sec, msg->header.stamp.nanosec);
  record.receive_stamp_us = this->get_clock()->now().nanoseconds() / 1000LL;
  record.frame_id = msg->header.frame_id;
  record.width = msg->width;
  record.height = msg->height;
  record.encoding = msg->encoding;
  record.step = msg->step;
  record.data_size = msg->data.size();

  const std::vector<uint8_t> bytes = save_images_ ? msg->data : std::vector<uint8_t>{};
  if (!core_->record(std::move(record), bytes)) {
    RCLCPP_WARN(get_logger(), "Record received after collection stopped for %s", spec.topic.c_str());
  }
}
```

For `use_sensor_data_qos=true`, create `rclcpp::SensorDataQoS()` and apply `.keep_last(qos_depth)`. Otherwise use `rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable()`.

- [ ] **Step 4: Implement timer shutdown and per-stream report**

If `duration_sec > 0`, create a wall timer. In its callback:

```cpp
void finish_collection()
{
  if (finished_.exchange(true)) {
    return;
  }
  for (auto & subscription : image_subscriptions_) {
    subscription.reset();
  }
  image_subscriptions_.clear();
  core_->stop();

  for (const auto & spec : subscriptions_) {
    RCLCPP_INFO(get_logger(), "%s/%s: %llu frames", spec.camera_name.c_str(), spec.stream_type.c_str(),
      static_cast<unsigned long long>(spec.frame_index));
  }
  RCLCPP_INFO(get_logger(), "CSV: %s/timestamps.csv", core_->capture_directory().string().c_str());
  RCLCPP_INFO(get_logger(), "Dropped image-save jobs: %llu",
    static_cast<unsigned long long>(core_->dropped_image_jobs()));
  rclcpp::shutdown();
}
```

For `duration_sec == 0`, retain subscriptions until Ctrl-C; register an `on_shutdown` callback in `main` or ensure the node destructor invokes `core_->stop()` so CSV data is flushed.

- [ ] **Step 5: Add executable target and run build**

Append to `OB ROS2/CMakeLists.txt` before the test block:

```cmake
add_executable(timestamp_collector_node src/timestamp_collector_node.cpp)
target_link_libraries(timestamp_collector_node collector_core)
target_include_directories(timestamp_collector_node PRIVATE include)
ament_target_dependencies(timestamp_collector_node rclcpp sensor_msgs)

install(TARGETS collector_core timestamp_collector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

Run:

```bash
colcon build --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
```

Expected: the package builds and produces `timestamp_collector_node`.

- [ ] **Step 6: Verify startup validation manually**

Run:

```bash
source install/setup.bash
ros2 run ob_ros2_timestamp_collector timestamp_collector_node --ros-args -p subscriptions:="[]"
```

Expected: fatal startup error that `subscriptions` must contain at least one item; no capture directory should be created.

- [ ] **Step 7: Commit the ROS node**

```bash
git add "OB ROS2/src/timestamp_collector_node.cpp" "OB ROS2/CMakeLists.txt"
git commit -m "feat: add ROS2 image timestamp collector node"
```

---

### Task 4: Add launchable two-camera configuration and operator documentation

**Files:**
- Create: `OB ROS2/config/collector.yaml`
- Create: `OB ROS2/launch/timestamp_collector.launch.py`
- Create: `OB ROS2/README.md`
- Modify: `OB ROS2/CMakeLists.txt`

**Interfaces:**
- Produces `ros2 launch ob_ros2_timestamp_collector timestamp_collector.launch.py`.
- The launch file starts only `timestamp_collector_node`; it does not include any Orbbec camera launch file.
- Provides a four-topic two-camera YAML example, configured with `save_images: false` by default.

- [ ] **Step 1: Add example collector parameters**

Create `OB ROS2/config/collector.yaml`:

```yaml
/**:
  ros__parameters:
    # The collector only subscribes. Start Orbbec cameras separately.
    # Keep camera names and topics aligned with the actual Orbbec launch configuration.
    subscriptions:
      - "camera_01|depth|/camera_01/depth/image_raw"
      - "camera_01|color|/camera_01/color/image_raw"
      - "camera_02|depth|/camera_02/depth/image_raw"
      - "camera_02|color|/camera_02/color/image_raw"

    # Parent directory; each run creates capture_YYYYmmdd_HHMMSS below it.
    output_dir: "/tmp/ob_ros2_capture"
    duration_sec: 30
    max_frames_per_stream: 0

    # SensorDataQoS normally matches image drivers that publish best-effort.
    qos_depth: 10
    use_sensor_data_qos: true

    # Raw bytes are stored unchanged as .bin only when this is true.
    save_images: false
    writer_queue_capacity: 64
    flush_every_n_records: 1
```

- [ ] **Step 2: Add the collector-only launch file**

Create `OB ROS2/launch/timestamp_collector.launch.py`:

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("ob_ros2_timestamp_collector")
    default_config = f"{package_share}/config/collector.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the timestamp collector YAML parameter file.",
        ),
        Node(
            package="ob_ros2_timestamp_collector",
            executable="timestamp_collector_node",
            name="timestamp_collector_node",
            output="screen",
            parameters=[LaunchConfiguration("config_file")],
        ),
    ])
```

- [ ] **Step 3: Install configuration and launch files**

Add to `OB ROS2/CMakeLists.txt` after the `install(TARGETS ...)` section:

```cmake
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
```

- [ ] **Step 4: Write README with exact camera-side requirements and runtime commands**

Create `OB ROS2/README.md` containing these sections and commands:

```markdown
# OB ROS2 Timestamp Collector

## Purpose

This package subscribes to already-running ROS 2 `sensor_msgs/msg/Image` topics. It writes raw timestamp and image metadata to CSV and can optionally save the image byte buffers. It does not launch cameras, match frames, measure synchronization quality, or generate hardware trigger signals.

## Camera configuration

For `header.stamp` to represent a timestamp that can later be compared across Orbbec cameras, start the Orbbec driver separately with:

```text
sync_mode:=hardware_triggering
time_domain:=global
enable_sync_host_time:=false
frames_per_trigger:=1
depth_delay_us:=0
color_delay_us:=0
trigger2image_delay_us:=0
```

Hardware wiring and the external trigger source must already be working. In a shared external-trigger topology, do not enable `trigger_out_enabled` on every camera unless the wiring intentionally forwards camera trigger output.

## Build

```bash
mkdir -p ~/ros2_ws/src
cp -a "OB ROS2" ~/ros2_ws/src/ob_ros2_timestamp_collector
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ob_ros2_timestamp_collector
source install/setup.bash
```

## Configure topics

Edit `config/collector.yaml`. Each subscription uses `camera_name|stream_type|topic`; valid stream types are `depth` and `color`.

## Run

Start the Orbbec camera launch in one terminal. Then run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ob_ros2_timestamp_collector timestamp_collector.launch.py
```

Override the YAML path with:

```bash
ros2 launch ob_ros2_timestamp_collector timestamp_collector.launch.py config_file:=/path/to/collector.yaml
```

## Output

Each run creates `capture_YYYYmmdd_HHMMSS/timestamps.csv` below `output_dir`. The CSV columns are:

```text
camera_name,stream_type,frame_index,header_stamp_us,receive_stamp_us,frame_id,width,height,encoding,step,data_size,image_path
```

`header_stamp_us` comes from `Image.header.stamp`. `receive_stamp_us` is when this node received the message and is useful for observing ROS transport and scheduling, not for judging camera exposure synchronization. `frame_index` is created by this collector per configured stream; `frame_id` is the ROS coordinate-frame identifier.

With `save_images: true`, raw bytes are stored as `images/<camera>/<stream>/<frame_index>.bin`. Use the CSV `encoding`, `width`, `height`, `step`, and `data_size` columns to interpret the bytes correctly.
```

- [ ] **Step 5: Build and inspect launch registration**

Run:

```bash
colcon build --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
source install/setup.bash
ros2 pkg executables ob_ros2_timestamp_collector
ros2 launch ob_ros2_timestamp_collector timestamp_collector.launch.py --show-args
```

Expected: `timestamp_collector_node` appears in package executables, and launch help lists `config_file`.

- [ ] **Step 6: Commit the operator-facing package files**

```bash
git add "OB ROS2/config/collector.yaml" "OB ROS2/launch/timestamp_collector.launch.py" "OB ROS2/README.md" "OB ROS2/CMakeLists.txt"
git commit -m "docs: add ROS2 timestamp collector configuration and usage"
```

---

### Task 5: End-to-end topic smoke test and final package validation

**Files:**
- Modify: `OB ROS2/README.md` only if a command or observed behavior differs from the implementation.

**Interfaces:**
- Consumes installed `timestamp_collector_node` and launch file.
- Validates the CSV output contract against a live ROS 2 image message.

- [ ] **Step 1: Build the package and run all unit tests**

Run:

```bash
colcon build --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
colcon test --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build succeeds and every gtest passes.

- [ ] **Step 2: Create a temporary single-topic collection configuration**

Run:

```bash
cat > /tmp/ob_ros2_smoke.yaml <<'EOF'
/**:
  ros__parameters:
    subscriptions:
      - "test_camera|color|/test_camera/color/image_raw"
    output_dir: "/tmp/ob_ros2_capture_smoke"
    duration_sec: 2
    max_frames_per_stream: 0
    qos_depth: 10
    use_sensor_data_qos: true
    save_images: true
    writer_queue_capacity: 4
    flush_every_n_records: 1
EOF
```

- [ ] **Step 3: Start the collector and publish a compatible test image**

In terminal A:

```bash
source install/setup.bash
ros2 launch ob_ros2_timestamp_collector timestamp_collector.launch.py config_file:=/tmp/ob_ros2_smoke.yaml
```

In terminal B, while the collector is active:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /test_camera/color/image_raw sensor_msgs/msg/Image "{header: {stamp: {sec: 10, nanosec: 500000000}, frame_id: test_frame}, height: 1, width: 3, encoding: rgb8, is_bigendian: 0, step: 3, data: [1, 2, 3]}"
```

Expected: after two seconds the collector prints one `test_camera/color` frame, the capture directory, and zero or more writer queue drops.

- [ ] **Step 4: Verify CSV and raw byte output**

Run:

```bash
latest=$(ls -dt /tmp/ob_ros2_capture_smoke/capture_* | head -1)
cat "$latest/timestamps.csv"
stat -c '%s' "$latest/images/test_camera/color/00000000.bin"
```

Expected: the CSV contains one row with `test_camera,color,0,10500000`, `rgb8`, dimensions `3,1`, `data_size` `3`, and image path `images/test_camera/color/00000000.bin`; `stat` prints `3`.

- [ ] **Step 5: Update README only if smoke-test output differs and run final test suite**

If exact output paths or commands differ, revise the relevant README section. Then run:

```bash
colcon test --packages-select ob_ros2_timestamp_collector --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all tests pass after documentation alignment.

- [ ] **Step 6: Commit any smoke-test-driven documentation correction**

```bash
git add "OB ROS2/README.md"
git commit -m "docs: verify timestamp collector smoke test workflow"
```

If no README changes were necessary, do not create an empty commit.

---

## Plan Self-Review

- **Spec coverage:** Tasks 1–2 implement timestamp/metadata records, CSV, raw byte saving, capture directory creation, locking, flushing, and bounded asynchronous image writes. Task 3 implements strict topic configuration, QoS, subscriptions, timing, limits, and reporting. Task 4 implements the collector-only launch file, example configuration, required Orbbec global-time guidance, and user documentation. Task 5 verifies the output against a live ROS image message.
- **Out-of-scope compliance:** No task introduces frame pairing, comparison statistics, camera-node startup, Orbbec SDK C++ dependency, or hardware-trigger/GPIO control.
- **Type consistency:** `CaptureRecord` and `CollectorCore` are defined in Tasks 1–2 before Task 3 consumes them. The parameter names used by Task 3 are exactly the YAML keys introduced in Task 4.
- **Placeholder scan:** No deferred implementation steps, unspecified validation, or undefined interfaces remain.
