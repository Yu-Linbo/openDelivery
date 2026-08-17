#ifndef RELOCALIZATION__RELOCALIZATION_CORE_HPP_
#define RELOCALIZATION__RELOCALIZATION_CORE_HPP_

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace relocalization {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ScanData {
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
  Pose2D laser_in_base;
  std::vector<float> ranges;
};

struct GridMap {
  uint32_t width{0};
  uint32_t height{0};
  double resolution{0.0};
  Pose2D origin;
  std::vector<int8_t> cells;
  bool valid() const;
};

struct ScanRecord {
  std::string id;
  std::string map_name;
  Pose2D pose;
  ScanData scan;
};

struct MatchConfig {
  double search_xy{1.0};
  double search_yaw{0.35};
  double coarse_xy_step{0.20};
  double coarse_yaw_step{0.0872664626};
  double refine_xy_step{0.05};
  double refine_yaw_step{0.0174532925};
  double correlation_resolution{0.10};
  double correlation_distance{0.20};
  double map_weight{0.65};
  int occupied_threshold{65};
  size_t max_beams{240};
};

struct MatchResult {
  bool valid{false};
  Pose2D pose;
  double score{0.0};
  double map_score{0.0};
  double scan_score{0.0};
};

bool valid_record_id(const std::string & id);
bool save_record(const std::string & path, const ScanRecord & record, std::string * error);
bool load_record(const std::string & path, ScanRecord * record, std::string * error);
std::vector<ScanRecord> load_records(
  const std::string & directory, const std::string & map_name,
  const std::string & points_json = "");

double map_match_score(
  const GridMap & map, const ScanData & scan, const Pose2D & pose,
  const MatchConfig & config);
double scan_match_score(
  const ScanData & current_scan, const Pose2D & current_pose,
  const ScanRecord & reference, const MatchConfig & config);
MatchResult search_pose(
  const GridMap & map, const ScanData & scan, const Pose2D & seed,
  const ScanRecord * reference, const MatchConfig & config);

}  // namespace relocalization

#endif  // RELOCALIZATION__RELOCALIZATION_CORE_HPP_
