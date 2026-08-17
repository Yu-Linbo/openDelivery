#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

#include "gtest/gtest.h"
#include "relocalization/relocalization_core.hpp"

namespace relocalization {
namespace {

GridMap wall_map() {
  GridMap map;
  map.width = 100;
  map.height = 100;
  map.resolution = 0.1;
  map.cells.assign(map.width * map.height, 0);
  for (uint32_t y = 0; y < map.height; ++y) {
    map.cells[y * map.width + 50] = 100;
  }
  return map;
}

ScanData forward_scan() {
  ScanData scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.01;
  scan.range_min = 0.05;
  scan.range_max = 20.0;
  scan.ranges = {2.0f};
  return scan;
}

TEST(RelocalizationCore, MapScoreAndSearchFindWall) {
  const GridMap map = wall_map();
  const ScanData scan = forward_scan();
  MatchConfig config;
  config.search_xy = 1.0;
  config.search_yaw = 0.01;
  config.coarse_xy_step = 0.2;
  config.coarse_yaw_step = 0.02;
  config.refine_xy_step = 0.05;
  config.refine_yaw_step = 0.01;
  EXPECT_DOUBLE_EQ(map_match_score(map, scan, Pose2D{3.0, 4.0, 0.0}, config), 1.0);
  EXPECT_DOUBLE_EQ(map_match_score(map, scan, Pose2D{2.0, 4.0, 0.0}, config), 0.0);
  const MatchResult result = search_pose(map, scan, Pose2D{2.4, 4.0, 0.0}, nullptr, config);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.pose.x, 3.0, 0.06);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
}

TEST(RelocalizationCore, HistoricalScanCorrelationPrefersRecordedPose) {
  const ScanData scan = forward_scan();
  ScanRecord record;
  record.id = "door_a";
  record.map_name = "floor_a";
  record.pose = Pose2D{3.0, 4.0, 0.0};
  record.scan = scan;
  MatchConfig config;
  EXPECT_DOUBLE_EQ(scan_match_score(scan, record.pose, record, config), 1.0);
  EXPECT_DOUBLE_EQ(scan_match_score(scan, Pose2D{1.0, 4.0, 0.0}, record, config), 0.0);
}

TEST(RelocalizationCore, RecordRoundTripAndMapFilter) {
  char directory_template[] = "/tmp/open_delivery_relocalization_XXXXXX";
  char * directory = mkdtemp(directory_template);
  ASSERT_NE(directory, nullptr);
  ScanRecord input;
  input.id = "startup_1";
  input.map_name = "test_101";
  input.pose = Pose2D{1.25, -2.5, 0.4};
  input.scan = forward_scan();
  input.scan.laser_in_base = Pose2D{0.2, 0.0, 0.1};
  const std::string path = std::string(directory) + "/startup_1.rloc";
  std::string error;
  ASSERT_TRUE(save_record(path, input, &error)) << error;
  ScanRecord output;
  ASSERT_TRUE(load_record(path, &output, &error)) << error;
  EXPECT_EQ(output.id, input.id);
  EXPECT_EQ(output.map_name, input.map_name);
  EXPECT_DOUBLE_EQ(output.pose.x, input.pose.x);
  ASSERT_EQ(output.scan.ranges.size(), 1u);
  EXPECT_FLOAT_EQ(output.scan.ranges[0], 2.0f);
  EXPECT_EQ(load_records(directory, "test_101").size(), 1u);
  EXPECT_TRUE(load_records(directory, "test_102").empty());
  unlink(path.c_str());
  rmdir(directory);
}

TEST(RelocalizationCore, RecordIdValidation) {
  EXPECT_TRUE(valid_record_id("door-A_2"));
  EXPECT_FALSE(valid_record_id(""));
  EXPECT_FALSE(valid_record_id("中文"));
  EXPECT_FALSE(valid_record_id("_leading"));
}

TEST(RelocalizationCore, PointJsonRemovalDeletesOrphanedRecord) {
  char directory_template[] = "/tmp/open_delivery_relocalization_points_XXXXXX";
  char * directory = mkdtemp(directory_template);
  ASSERT_NE(directory, nullptr);
  ScanRecord keep;
  keep.id = "keep";
  keep.map_name = "floor1";
  keep.pose = Pose2D{1.0, 2.0, 0.3};
  keep.scan = forward_scan();
  ScanRecord removed = keep;
  removed.id = "removed";
  const std::string keep_path = std::string(directory) + "/keep.rloc";
  const std::string removed_path = std::string(directory) + "/removed.rloc";
  const std::string points_path = std::string(directory) + "/floor1_points.json";
  std::string error;
  ASSERT_TRUE(save_record(keep_path, keep, &error)) << error;
  ASSERT_TRUE(save_record(removed_path, removed, &error)) << error;
  {
    std::ofstream points(points_path);
    points << R"({"map":"floor1","points":[{"id":"keep","type":"relocalization","x":1,"y":2,"yaw":0.3}]})";
  }

  const auto records = load_records(directory, "floor1", points_path);
  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(records.front().id, "keep");
  EXPECT_EQ(access(keep_path.c_str(), F_OK), 0);
  EXPECT_NE(access(removed_path.c_str(), F_OK), 0);
  unlink(keep_path.c_str());
  unlink(points_path.c_str());
  rmdir(directory);
}

}  // namespace
}  // namespace relocalization
