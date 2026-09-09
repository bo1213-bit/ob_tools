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
    std::size_t writer_queue_capacity{64};
    std::size_t flush_every_n_records{1};
  };

  explicit CollectorCore(Config config);
  ~CollectorCore();

  CollectorCore(const CollectorCore &) = delete;
  CollectorCore & operator=(const CollectorCore &) = delete;

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
  std::size_t records_since_flush_{0};

  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::queue<ImageJob> image_jobs_;
  std::thread writer_thread_;
  bool writer_running_{false};

  std::atomic<bool> accepting_records_{false};
  std::atomic<uint64_t> dropped_image_jobs_{0};
};

}  // namespace ob_ros2_timestamp_collector
