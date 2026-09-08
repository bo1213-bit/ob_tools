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
