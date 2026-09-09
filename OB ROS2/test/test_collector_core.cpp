#include "ob_ros2_timestamp_collector/collector_core.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <gtest/gtest.h>
#include <string>

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

  collector::CaptureRecord record{
    "camera_01", "color", 0, 100, 200, "camera_color", 640, 480,
    "rgb8", 1920, 921600, ""};
  EXPECT_TRUE(core.record(record, {}));
  core.stop();

  std::ifstream csv(core.capture_directory() / "timestamps.csv");
  std::string header;
  std::string line;
  std::getline(csv, header);
  std::getline(csv, line);
  EXPECT_EQ(
    header,
    "camera_name,stream_type,frame_index,header_stamp_us,receive_stamp_us,frame_id,width,height,encoding,step,data_size,image_path");
  EXPECT_NE(line.find("camera_01,color,0,100,200"), std::string::npos);
}

TEST_F(CollectorCoreTest, SavesOriginalBytesAndWritesRelativePath)
{
  collector::CollectorCore core({root_, true, 2, 1});
  core.start();

  collector::CaptureRecord record{
    "camera_01", "depth", 3, 100, 200, "camera_depth", 2, 2,
    "16UC1", 4, 8, ""};
  EXPECT_TRUE(core.record(record, {1, 2, 3, 4, 5, 6, 7, 8}));
  core.stop();

  const auto image_path = core.capture_directory() / "images/camera_01/depth/00000003.bin";
  EXPECT_TRUE(std::filesystem::exists(image_path));
  EXPECT_EQ(std::filesystem::file_size(image_path), 8U);

  std::ifstream image(image_path, std::ios::binary);
  const std::vector<unsigned char> bytes{
    std::istreambuf_iterator<char>(image), std::istreambuf_iterator<char>()};
  EXPECT_EQ(bytes, (std::vector<unsigned char>{1, 2, 3, 4, 5, 6, 7, 8}));
}
