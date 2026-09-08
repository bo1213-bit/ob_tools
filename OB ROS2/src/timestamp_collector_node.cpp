#include "ob_ros2_timestamp_collector/collector_core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ob_ros2_timestamp_collector
{
namespace
{

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
  if (
    first == std::string::npos || second == std::string::npos ||
    value.find('|', second + 1) != std::string::npos)
  {
    throw std::invalid_argument("subscription must use camera_name|stream_type|topic: " + value);
  }

  SubscriptionSpec result{
    value.substr(0, first),
    value.substr(first + 1, second - first - 1),
    value.substr(second + 1)};

  if (result.camera_name.empty() || result.topic.empty()) {
    throw std::invalid_argument("camera_name and topic must not be empty: " + value);
  }
  if (result.stream_type != "color" && result.stream_type != "depth") {
    throw std::invalid_argument("stream_type must be color or depth: " + result.stream_type);
  }

  return result;
}

class TimestampCollectorNode : public rclcpp::Node
{
public:
  TimestampCollectorNode()
  : Node("timestamp_collector_node")
  {
    const auto subscription_values = declare_parameter<std::vector<std::string>>("subscriptions", {});
    const auto output_dir = declare_parameter<std::string>("output_dir", "/tmp/ob_ros2_capture");
    const auto duration_sec = declare_parameter<int64_t>("duration_sec", 30);
    max_frames_per_stream_ = declare_parameter<int64_t>("max_frames_per_stream", 0);
    const auto qos_depth = declare_parameter<int64_t>("qos_depth", 10);
    const auto use_sensor_data_qos = declare_parameter<bool>("use_sensor_data_qos", true);
    save_images_ = declare_parameter<bool>("save_images", false);
    const auto writer_queue_capacity = declare_parameter<int64_t>("writer_queue_capacity", 64);
    const auto flush_every_n_records = declare_parameter<int64_t>("flush_every_n_records", 1);

    if (subscription_values.empty()) {
      throw std::invalid_argument(
              "subscriptions must contain at least one camera_name|stream_type|topic item");
    }
    if (output_dir.empty()) {
      throw std::invalid_argument("output_dir must not be empty");
    }
    if (
      duration_sec < 0 || max_frames_per_stream_ < 0 || qos_depth <= 0 ||
      writer_queue_capacity <= 0 || flush_every_n_records <= 0)
    {
      throw std::invalid_argument(
              "duration, frame limit, QoS depth, queue capacity, and flush count must be valid "
              "non-negative/positive values");
    }

    std::unordered_set<std::string> subscription_keys;
    for (const auto & value : subscription_values) {
      auto spec = parse_subscription_spec(value);
      const auto key = spec.camera_name + "|" + spec.stream_type;
      if (!subscription_keys.insert(key).second) {
        throw std::invalid_argument("duplicate camera_name|stream_type subscription: " + key);
      }
      subscriptions_.push_back(std::move(spec));
    }

    CollectorCore::Config core_config;
    core_config.output_dir = output_dir;
    core_config.save_images = save_images_;
    core_config.writer_queue_capacity = static_cast<std::size_t>(writer_queue_capacity);
    core_config.flush_every_n_records = static_cast<std::size_t>(flush_every_n_records);
    core_ = std::make_unique<CollectorCore>(std::move(core_config));
    core_->start();

    rclcpp::QoS qos = use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(rclcpp::KeepLast(1));
    qos.keep_last(static_cast<std::size_t>(qos_depth));
    if (!use_sensor_data_qos) {
      qos.reliable();
    }

    image_subscriptions_.reserve(subscriptions_.size());
    for (std::size_t index = 0; index < subscriptions_.size(); ++index) {
      image_subscriptions_.push_back(create_subscription<sensor_msgs::msg::Image>(
        subscriptions_[index].topic,
        qos,
        [this, index](const sensor_msgs::msg::Image::ConstSharedPtr message) {
          image_callback(message, index);
        }));
      RCLCPP_INFO(
        get_logger(), "Subscribing to %s as %s/%s", subscriptions_[index].topic.c_str(),
        subscriptions_[index].camera_name.c_str(), subscriptions_[index].stream_type.c_str());
    }

    if (duration_sec > 0) {
      finish_timer_ = create_wall_timer(
        std::chrono::seconds(duration_sec),
        std::bind(&TimestampCollectorNode::finish_collection, this));
    }

    RCLCPP_INFO(
      get_logger(), "Writing capture data to %s", core_->capture_directory().string().c_str());
  }

  ~TimestampCollectorNode() override
  {
    if (core_) {
      core_->stop();
    }
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message, const std::size_t spec_index)
  {
    if (finished_.load()) {
      return;
    }

    auto & spec = subscriptions_.at(spec_index);
    if (
      max_frames_per_stream_ > 0 &&
      spec.frame_index >= static_cast<uint64_t>(max_frames_per_stream_))
    {
      return;
    }

    CaptureRecord record;
    record.camera_name = spec.camera_name;
    record.stream_type = spec.stream_type;
    record.frame_index = spec.frame_index++;
    record.header_stamp_us = stamp_to_microseconds(message->header.stamp.sec, message->header.stamp.nanosec);
    record.receive_stamp_us = get_clock()->now().nanoseconds() / 1000LL;
    record.frame_id = message->header.frame_id;
    record.width = message->width;
    record.height = message->height;
    record.encoding = message->encoding;
    record.step = message->step;
    record.data_size = message->data.size();

    const std::vector<uint8_t> image_bytes = save_images_ ? message->data : std::vector<uint8_t>{};
    if (!core_->record(std::move(record), image_bytes)) {
      RCLCPP_WARN(get_logger(), "Record received after collection stopped for %s", spec.topic.c_str());
    }
  }

  void finish_collection()
  {
    if (finished_.exchange(true)) {
      return;
    }

    if (finish_timer_) {
      finish_timer_->cancel();
    }
    image_subscriptions_.clear();
    core_->stop();

    for (const auto & spec : subscriptions_) {
      RCLCPP_INFO(
        get_logger(), "%s/%s: %llu frames", spec.camera_name.c_str(), spec.stream_type.c_str(),
        static_cast<unsigned long long>(spec.frame_index));
    }
    RCLCPP_INFO(get_logger(), "CSV: %s/timestamps.csv", core_->capture_directory().string().c_str());
    RCLCPP_INFO(
      get_logger(), "Dropped image-save jobs: %llu",
      static_cast<unsigned long long>(core_->dropped_image_jobs()));
    rclcpp::shutdown();
  }

  std::unique_ptr<CollectorCore> core_;
  std::vector<SubscriptionSpec> subscriptions_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscriptions_;
  rclcpp::TimerBase::SharedPtr finish_timer_;
  int64_t max_frames_per_stream_{0};
  bool save_images_{false};
  std::atomic<bool> finished_{false};
};

}  // namespace

std::shared_ptr<rclcpp::Node> make_timestamp_collector_node()
{
  return std::make_shared<TimestampCollectorNode>();
}

}  // namespace ob_ros2_timestamp_collector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(ob_ros2_timestamp_collector::make_timestamp_collector_node());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("timestamp_collector_node"), "Startup failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
