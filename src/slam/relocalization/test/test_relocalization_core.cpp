#include <cstdlib>
#include <fstream>
#include <string>
#include <sys/stat.h>
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

TEST(RelocalizationCore, DiscoversRecordMapsWithCurrentMapFirst) {
  char root_template[] = "/tmp/open_delivery_relocalization_maps_XXXXXX";
  char * directory = mkdtemp(root_template);
  ASSERT_NE(directory, nullptr);
  const std::string root(directory);
  auto make_map = [&root](const std::string & name, bool with_record) {
    const std::string folder = root + "/" + name;
    const std::string records = folder + "/relocalization";
    ASSERT_EQ(mkdir(folder.c_str(), 0755), 0);
    ASSERT_EQ(mkdir(records.c_str(), 0755), 0);
    std::ofstream(folder + "/" + name + ".yaml") << "image: " << name << ".pgm\n";
    if (with_record) {
      std::ofstream(records + "/point.rloc") << "placeholder";
    }
  };
  make_map("floor_a", true);
  make_map("floor_b", true);
  make_map("floor_empty", false);

  const auto maps = discover_record_maps(root, "floor_b");
  ASSERT_EQ(maps.size(), 2u);
  EXPECT_EQ(maps[0], "floor_b");
  EXPECT_EQ(maps[1], "floor_a");

  for (const std::string name : {"floor_a", "floor_b", "floor_empty"}) {
    const std::string folder = root + "/" + name;
    unlink((folder + "/relocalization/point.rloc").c_str());
    unlink((folder + "/" + name + ".yaml").c_str());
    rmdir((folder + "/relocalization").c_str());
    rmdir(folder.c_str());
  }
  rmdir(root.c_str());
}

TEST(RelocalizationCore, LoadsMapYamlAndFlipsPgmRowsIntoMapCoordinates) {
  char root_template[] = "/tmp/open_delivery_relocalization_grid_XXXXXX";
  char * directory = mkdtemp(root_template);
  ASSERT_NE(directory, nullptr);
  const std::string root(directory);
  const std::string yaml_path = root + "/floor.yaml";
  const std::string image_path = root + "/floor.pgm";
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: floor.pgm\n"
         << "resolution: 0.25\n"
         << "origin: [1.0, -2.0, 0.5]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
  }
  {
    std::ofstream pgm(image_path, std::ios::binary);
    pgm << "P5\n# top row then bottom row\n2 2\n255\n";
    const unsigned char pixels[] = {0, 255, 128, 255};
    pgm.write(reinterpret_cast<const char *>(pixels), sizeof(pixels));
  }

  GridMap map;
  std::string error;
  ASSERT_TRUE(load_grid_map_from_yaml(yaml_path, &map, &error)) << error;
  ASSERT_TRUE(map.valid());
  EXPECT_EQ(map.width, 2u);
  EXPECT_EQ(map.height, 2u);
  EXPECT_DOUBLE_EQ(map.resolution, 0.25);
  EXPECT_DOUBLE_EQ(map.origin.x, 1.0);
  EXPECT_DOUBLE_EQ(map.origin.y, -2.0);
  EXPECT_DOUBLE_EQ(map.origin.yaw, 0.5);
  ASSERT_EQ(map.cells.size(), 4u);
  EXPECT_EQ(static_cast<int>(map.cells[0]), -1);
  EXPECT_EQ(static_cast<int>(map.cells[1]), 0);
  EXPECT_EQ(static_cast<int>(map.cells[2]), 100);
  EXPECT_EQ(static_cast<int>(map.cells[3]), 0);

  unlink(yaml_path.c_str());
  unlink(image_path.c_str());
  rmdir(root.c_str());
}

}  // namespace
}  // namespace relocalization
