#include "ob_ros2_timestamp_collector/collector_core.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

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

CollectorCore::CollectorCore(Config config)
: config_(std::move(config))
{
}

CollectorCore::~CollectorCore()
{
  stop();
}

void CollectorCore::start()
{
  if (accepting_records_.load()) {
    throw std::logic_error("CollectorCore has already started");
  }
  if (config_.output_dir.empty()) {
    throw std::invalid_argument("output_dir must not be empty");
  }
  if (config_.save_images && config_.writer_queue_capacity == 0U) {
    throw std::invalid_argument("writer_queue_capacity must be greater than zero when saving images");
  }
  if (config_.flush_every_n_records == 0U) {
    throw std::invalid_argument("flush_every_n_records must be greater than zero");
  }

  const auto base_directory = config_.output_dir / capture_directory_name();
  capture_directory_ = base_directory;
  for (uint32_t suffix = 1; std::filesystem::exists(capture_directory_); ++suffix) {
    capture_directory_ = base_directory.string() + "_" + std::to_string(suffix);
  }
  std::filesystem::create_directories(capture_directory_);

  csv_file_.open(capture_directory_ / "timestamps.csv", std::ios::out | std::ios::trunc);
  if (!csv_file_.is_open()) {
    throw std::runtime_error("cannot open timestamps.csv");
  }

  csv_file_ << "camera_name,stream_type,frame_index,header_stamp_us,receive_stamp_us,frame_id,width,height,encoding,step,data_size,image_path\n";
  csv_file_.flush();

  if (config_.save_images) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      writer_running_ = true;
    }
    writer_thread_ = std::thread(&CollectorCore::writer_loop, this);
  }

  accepting_records_.store(true);
}

bool CollectorCore::record(CaptureRecord record, std::vector<uint8_t> image_bytes)
{
  if (!accepting_records_.load()) {
    return false;
  }

  if (config_.save_images) {
    const auto relative_path = image_relative_path(record);
    bool queued = false;

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (writer_running_ && image_jobs_.size() < config_.writer_queue_capacity) {
        image_jobs_.push(ImageJob{capture_directory_ / relative_path, std::move(image_bytes)});
        queued = true;
      }
    }

    if (queued) {
      record.image_path = relative_path.generic_string();
      queue_condition_.notify_one();
    } else {
      record.image_path.clear();
      ++dropped_image_jobs_;
    }
  }

  write_csv_record(record);
  return true;
}

void CollectorCore::stop()
{
  accepting_records_.store(false);

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

std::filesystem::path CollectorCore::capture_directory() const
{
  return capture_directory_;
}

uint64_t CollectorCore::dropped_image_jobs() const
{
  return dropped_image_jobs_.load();
}

std::filesystem::path CollectorCore::image_relative_path(const CaptureRecord & record) const
{
  std::ostringstream filename;
  filename << std::setw(8) << std::setfill('0') << record.frame_index << ".bin";
  return std::filesystem::path{"images"} / record.camera_name / record.stream_type / filename.str();
}

void CollectorCore::write_csv_record(const CaptureRecord & record)
{
  std::lock_guard<std::mutex> lock(csv_mutex_);
  if (!csv_file_.is_open()) {
    return;
  }

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

    std::error_code error;
    std::filesystem::create_directories(job.path.parent_path(), error);
    if (error) {
      std::cerr << "Failed to create image directory " << job.path.parent_path() << ": " << error.message() << '\n';
      continue;
    }

    std::ofstream image_file(job.path, std::ios::binary | std::ios::trunc);
    if (!image_file.is_open()) {
      std::cerr << "Failed to open image file " << job.path << '\n';
      continue;
    }

    if (!job.bytes.empty()) {
      image_file.write(
        reinterpret_cast<const char *>(job.bytes.data()),
        static_cast<std::streamsize>(job.bytes.size()));
      if (!image_file) {
        std::cerr << "Failed to write image file " << job.path << '\n';
      }
    }
  }
}

}  // namespace ob_ros2_timestamp_collector
