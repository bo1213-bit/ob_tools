#pragma once

#include <cstddef>
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
  std::size_t data_size{};
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
