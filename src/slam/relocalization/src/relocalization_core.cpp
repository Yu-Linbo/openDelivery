#include "relocalization/relocalization_core.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <json/json.h>
#include <limits>
#include <regex>
#include <sstream>
#include <sys/stat.h>
#include <unordered_set>

namespace relocalization {
namespace {

constexpr char kMagic[8] = {'O', 'D', 'R', 'L', 'O', 'C', '1', '\0'};
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double value) {
  while (value > kPi) value -= 2.0 * kPi;
  while (value < -kPi) value += 2.0 * kPi;
  return value;
}

template<typename T>
bool write_value(std::ofstream & stream, const T & value) {
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
  return stream.good();
}

template<typename T>
bool read_value(std::ifstream & stream, T * value) {
  stream.read(reinterpret_cast<char *>(value), sizeof(T));
  return stream.good();
}

bool write_string(std::ofstream & stream, const std::string & value) {
  const uint32_t size = static_cast<uint32_t>(value.size());
  if (!write_value(stream, size)) return false;
  stream.write(value.data(), size);
  return stream.good();
}

bool read_string(std::ifstream & stream, std::string * value) {
  uint32_t size = 0;
  if (!read_value(stream, &size) || size > 4096) return false;
  value->resize(size);
  if (size) stream.read(&(*value)[0], size);
  return stream.good();
}

bool usable_range(float range, const ScanData & scan) {
  return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

std::vector<std::pair<double, double>> endpoints(
  const ScanData & scan, const Pose2D & base_pose, size_t max_beams)
{
  std::vector<std::pair<double, double>> output;
  if (scan.ranges.empty() || scan.angle_increment == 0.0) return output;
  const size_t stride = std::max<size_t>(1, scan.ranges.size() / std::max<size_t>(1, max_beams));
  const double cb = std::cos(base_pose.yaw);
  const double sb = std::sin(base_pose.yaw);
  const double cl = std::cos(scan.laser_in_base.yaw);
  const double sl = std::sin(scan.laser_in_base.yaw);
  output.reserve(scan.ranges.size() / stride + 1);
  for (size_t index = 0; index < scan.ranges.size(); index += stride) {
    const float range = scan.ranges[index];
    if (!usable_range(range, scan)) continue;
    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const double laser_x = static_cast<double>(range) * std::cos(angle);
    const double laser_y = static_cast<double>(range) * std::sin(angle);
    const double base_x = scan.laser_in_base.x + cl * laser_x - sl * laser_y;
    const double base_y = scan.laser_in_base.y + sl * laser_x + cl * laser_y;
    output.emplace_back(
      base_pose.x + cb * base_x - sb * base_y,
      base_pose.y + sb * base_x + cb * base_y);
  }
  return output;
}

int64_t bin_key(int32_t x, int32_t y) {
  const uint64_t ux = static_cast<uint32_t>(x);
  const uint64_t uy = static_cast<uint32_t>(y);
  return static_cast<int64_t>((ux << 32) ^ uy);
}

double combined_score(
  const GridMap & map, const ScanData & scan, const Pose2D & pose,
  const ScanRecord * reference, const MatchConfig & config,
  double * map_score_out, double * scan_score_out)
{
  const double map_score = map_match_score(map, scan, pose, config);
  const double scan_score = reference ? scan_match_score(scan, pose, *reference, config) : 0.0;
  if (map_score_out) *map_score_out = map_score;
  if (scan_score_out) *scan_score_out = scan_score;
  return reference ? config.map_weight * map_score + (1.0 - config.map_weight) * scan_score
                   : map_score;
}

MatchResult search_window(
  const GridMap & map, const ScanData & scan, const Pose2D & center,
  double xy_radius, double yaw_radius, double xy_step, double yaw_step,
  const ScanRecord * reference, const MatchConfig & config)
{
  MatchResult best;
  if (xy_step <= 0.0 || yaw_step <= 0.0) return best;
  const int nx = static_cast<int>(std::ceil(xy_radius / xy_step));
  const int nyaw = static_cast<int>(std::ceil(yaw_radius / yaw_step));
  for (int ix = -nx; ix <= nx; ++ix) {
    for (int iy = -nx; iy <= nx; ++iy) {
      for (int ia = -nyaw; ia <= nyaw; ++ia) {
        Pose2D candidate{
          center.x + ix * xy_step,
          center.y + iy * xy_step,
          normalize_angle(center.yaw + ia * yaw_step)};
        double map_score = 0.0;
        double scan_score = 0.0;
        const double score = combined_score(
          map, scan, candidate, reference, config, &map_score, &scan_score);
        if (!best.valid || score > best.score) {
          best.valid = true;
          best.pose = candidate;
          best.score = score;
          best.map_score = map_score;
          best.scan_score = scan_score;
        }
      }
    }
  }
  return best;
}

std::string trim(const std::string & value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string unquote(const std::string & value) {
  if (value.size() >= 2 &&
    ((value.front() == '"' && value.back() == '"') ||
    (value.front() == '\'' && value.back() == '\'')))
  {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

std::string parent_directory(const std::string & path) {
  const auto separator = path.find_last_of('/');
  if (separator == std::string::npos) return ".";
  if (separator == 0) return "/";
  return path.substr(0, separator);
}

bool path_is_directory(const std::string & path) {
  struct stat info {};
  return ::stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool path_is_file(const std::string & path) {
  struct stat info {};
  return ::stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode);
}

bool has_record_file(const std::string & directory) {
  DIR * dir = opendir(directory.c_str());
  if (!dir) return false;
  bool found = false;
  while (dirent * entry = readdir(dir)) {
    const std::string name = entry->d_name;
    if (name.size() > 5 && name.substr(name.size() - 5) == ".rloc") {
      found = true;
      break;
    }
  }
  closedir(dir);
  return found;
}

bool parse_number(const std::string & value, double * output) {
  if (!output) return false;
  std::istringstream stream(trim(value));
  double number = 0.0;
  stream >> number;
  if (!stream || !std::isfinite(number)) return false;
  stream >> std::ws;
  if (!stream.eof()) return false;
  *output = number;
  return true;
}

bool parse_origin(const std::string & value, Pose2D * output) {
  if (!output) return false;
  std::string text = trim(value);
  if (text.size() < 2 || text.front() != '[' || text.back() != ']') return false;
  text = text.substr(1, text.size() - 2);
  std::replace(text.begin(), text.end(), ',', ' ');
  std::istringstream stream(text);
  Pose2D origin;
  stream >> origin.x >> origin.y >> origin.yaw;
  stream >> std::ws;
  if (!stream || !stream.eof() || !std::isfinite(origin.x) ||
    !std::isfinite(origin.y) || !std::isfinite(origin.yaw))
  {
    return false;
  }
  *output = origin;
  return true;
}

bool read_pgm_token(std::istream & stream, std::string * token) {
  while (stream >> *token) {
    if (!token->empty() && token->front() == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    return true;
  }
  return false;
}

}  // namespace

bool GridMap::valid() const {
  return width > 0 && height > 0 && resolution > 0.0 &&
    cells.size() == static_cast<size_t>(width) * height;
}

bool valid_record_id(const std::string & id) {
  static const std::regex pattern("^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$");
  return std::regex_match(id, pattern);
}

bool save_record(const std::string & path, const ScanRecord & record, std::string * error) {
  if (!valid_record_id(record.id) || record.map_name.empty() || record.scan.ranges.empty()) {
    if (error) *error = "invalid record id, map, or empty scan";
    return false;
  }
  const std::string temporary = path + ".tmp";
  std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
  if (!stream) {
    if (error) *error = "cannot open temporary record";
    return false;
  }
  stream.write(kMagic, sizeof(kMagic));
  bool ok = write_string(stream, record.id) && write_string(stream, record.map_name) &&
    write_value(stream, record.pose.x) && write_value(stream, record.pose.y) &&
    write_value(stream, record.pose.yaw) &&
    write_value(stream, record.scan.angle_min) &&
    write_value(stream, record.scan.angle_increment) &&
    write_value(stream, record.scan.range_min) && write_value(stream, record.scan.range_max) &&
    write_value(stream, record.scan.laser_in_base.x) &&
    write_value(stream, record.scan.laser_in_base.y) &&
    write_value(stream, record.scan.laser_in_base.yaw);
  const uint32_t count = static_cast<uint32_t>(record.scan.ranges.size());
  ok = ok && count <= 20000 && write_value(stream, count);
  if (ok) {
    stream.write(
      reinterpret_cast<const char *>(record.scan.ranges.data()),
      static_cast<std::streamsize>(count * sizeof(float)));
    ok = stream.good();
  }
  stream.close();
  if (!ok || std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    if (error) *error = "failed to serialize or atomically replace record";
    return false;
  }
  return true;
}

bool load_record(const std::string & path, ScanRecord * record, std::string * error) {
  if (!record) return false;
  std::ifstream stream(path, std::ios::binary);
  char magic[sizeof(kMagic)]{};
  if (!stream.read(magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    if (error) *error = "invalid record magic";
    return false;
  }
  ScanRecord output;
  bool ok = read_string(stream, &output.id) && read_string(stream, &output.map_name) &&
    read_value(stream, &output.pose.x) && read_value(stream, &output.pose.y) &&
    read_value(stream, &output.pose.yaw) &&
    read_value(stream, &output.scan.angle_min) &&
    read_value(stream, &output.scan.angle_increment) &&
    read_value(stream, &output.scan.range_min) && read_value(stream, &output.scan.range_max) &&
    read_value(stream, &output.scan.laser_in_base.x) &&
    read_value(stream, &output.scan.laser_in_base.y) &&
    read_value(stream, &output.scan.laser_in_base.yaw);
  uint32_t count = 0;
  ok = ok && read_value(stream, &count) && count > 0 && count <= 20000;
  if (ok) {
    output.scan.ranges.resize(count);
    stream.read(
      reinterpret_cast<char *>(output.scan.ranges.data()),
      static_cast<std::streamsize>(count * sizeof(float)));
    ok = stream.good();
  }
  if (!ok || !valid_record_id(output.id) || output.map_name.empty()) {
    if (error) *error = "invalid or truncated record";
    return false;
  }
  *record = std::move(output);
  return true;
}

std::vector<ScanRecord> load_records(
  const std::string & directory, const std::string & map_name,
  const std::string & points_json)
{
  bool reconcile = false;
  std::unordered_set<std::string> point_ids;
  if (!points_json.empty()) {
    std::ifstream points_stream(points_json);
    if (!points_stream) {
      reconcile = true;  // Missing points file means no relocalization records are referenced.
    } else {
      Json::Value root;
      Json::CharReaderBuilder builder;
      std::string parse_error;
      if (Json::parseFromStream(builder, points_stream, &root, &parse_error)) {
        const Json::Value rows = root.isArray() ? root : root["points"];
        if (rows.isArray()) {
          reconcile = true;
          for (const auto & row : rows) {
            if (row.isObject() && row["type"].asString() == "relocalization") {
              point_ids.insert(row["id"].asString());
            }
          }
        }
      }
    }
  }
  std::vector<std::string> paths;
  DIR * dir = opendir(directory.c_str());
  if (!dir) return {};
  while (dirent * entry = readdir(dir)) {
    const std::string name = entry->d_name;
    if (name.size() > 5 && name.substr(name.size() - 5) == ".rloc") {
      paths.push_back(directory + "/" + name);
    }
  }
  closedir(dir);
  std::sort(paths.begin(), paths.end());
  std::vector<ScanRecord> records;
  for (const auto & path : paths) {
    ScanRecord record;
    if (load_record(path, &record, nullptr) && record.map_name == map_name) {
      if (reconcile && !point_ids.count(record.id)) {
        std::remove(path.c_str());
        continue;
      }
      records.push_back(std::move(record));
    }
  }
  return records;
}

std::vector<std::string> discover_record_maps(
  const std::string & map_root, const std::string & current_map)
{
  std::vector<std::string> maps;
  DIR * root = opendir(map_root.c_str());
  if (!root) return maps;
  while (dirent * entry = readdir(root)) {
    const std::string name = entry->d_name;
    if (!valid_record_id(name)) continue;
    const std::string folder = map_root + "/" + name;
    if (!path_is_directory(folder) ||
      !path_is_file(folder + "/" + name + ".yaml") ||
      !has_record_file(folder + "/relocalization"))
    {
      continue;
    }
    maps.push_back(name);
  }
  closedir(root);
  std::sort(maps.begin(), maps.end());
  const auto current = std::find(maps.begin(), maps.end(), current_map);
  if (current != maps.end() && current != maps.begin()) {
    std::rotate(maps.begin(), current, current + 1);
  }
  return maps;
}

bool load_grid_map_from_yaml(
  const std::string & yaml_path, GridMap * map, std::string * error)
{
  if (!map) {
    if (error) *error = "map output is null";
    return false;
  }
  std::ifstream yaml(yaml_path);
  if (!yaml) {
    if (error) *error = "cannot open map yaml: " + yaml_path;
    return false;
  }
  std::string image;
  double resolution = 0.0;
  double occupied_threshold = 0.65;
  double free_threshold = 0.196;
  double negate_number = 0.0;
  Pose2D origin;
  bool have_origin = false;
  std::string line;
  while (std::getline(yaml, line)) {
    const auto separator = line.find(':');
    if (separator == std::string::npos) continue;
    const std::string key = trim(line.substr(0, separator));
    std::string value = trim(line.substr(separator + 1));
    const auto comment = value.find('#');
    if (comment != std::string::npos) value = trim(value.substr(0, comment));
    if (key == "image") {
      image = unquote(value);
    } else if (key == "resolution") {
      if (!parse_number(value, &resolution)) resolution = 0.0;
    } else if (key == "origin") {
      have_origin = parse_origin(value, &origin);
    } else if (key == "negate") {
      if (!parse_number(value, &negate_number)) negate_number = 0.0;
    } else if (key == "occupied_thresh") {
      if (!parse_number(value, &occupied_threshold)) occupied_threshold = -1.0;
    } else if (key == "free_thresh") {
      if (!parse_number(value, &free_threshold)) free_threshold = -1.0;
    }
  }
  if (image.empty() || resolution <= 0.0 || !have_origin ||
    occupied_threshold < 0.0 || occupied_threshold > 1.0 ||
    free_threshold < 0.0 || free_threshold > occupied_threshold)
  {
    if (error) *error = "invalid map yaml metadata: " + yaml_path;
    return false;
  }
  const std::string image_path =
    image.front() == '/' ? image : parent_directory(yaml_path) + "/" + image;
  std::ifstream pgm(image_path, std::ios::binary);
  std::string magic;
  std::string width_text;
  std::string height_text;
  std::string max_value_text;
  if (!pgm || !read_pgm_token(pgm, &magic) ||
    !read_pgm_token(pgm, &width_text) || !read_pgm_token(pgm, &height_text) ||
    !read_pgm_token(pgm, &max_value_text))
  {
    if (error) *error = "cannot read map image header: " + image_path;
    return false;
  }
  uint64_t width = 0;
  uint64_t height = 0;
  uint64_t max_value = 0;
  try {
    width = std::stoull(width_text);
    height = std::stoull(height_text);
    max_value = std::stoull(max_value_text);
  } catch (const std::exception &) {
    if (error) *error = "invalid map image dimensions: " + image_path;
    return false;
  }
  if ((magic != "P5" && magic != "P2") || width == 0 || height == 0 ||
    width > std::numeric_limits<uint32_t>::max() ||
    height > std::numeric_limits<uint32_t>::max() ||
    width * height > 100000000ULL || max_value == 0 || max_value > 255)
  {
    if (error) *error = "unsupported map image: " + image_path;
    return false;
  }
  if (magic == "P5") {
    char separator = 0;
    pgm.get(separator);
    if (!std::isspace(static_cast<unsigned char>(separator))) {
      if (error) *error = "invalid binary map image header: " + image_path;
      return false;
    }
    if (separator == '\r' && pgm.peek() == '\n') pgm.get();
  }
  GridMap output;
  output.width = static_cast<uint32_t>(width);
  output.height = static_cast<uint32_t>(height);
  output.resolution = resolution;
  output.origin = origin;
  output.cells.resize(static_cast<size_t>(width * height));
  const bool negate = std::abs(negate_number) > 0.5;
  for (uint64_t source_y = 0; source_y < height; ++source_y) {
    for (uint64_t x = 0; x < width; ++x) {
      int pixel = -1;
      if (magic == "P5") {
        pixel = pgm.get();
      } else {
        std::string pixel_text;
        if (read_pgm_token(pgm, &pixel_text)) {
          try {
            pixel = std::stoi(pixel_text);
          } catch (const std::exception &) {
            pixel = -1;
          }
        }
      }
      if (pixel < 0 || static_cast<uint64_t>(pixel) > max_value) {
        if (error) *error = "truncated or invalid map image: " + image_path;
        return false;
      }
      const double shade = static_cast<double>(pixel) / static_cast<double>(max_value);
      const double occupancy = negate ? shade : 1.0 - shade;
      const int8_t cell = occupancy > occupied_threshold ? 100 :
        (occupancy < free_threshold ? 0 : -1);
      const uint64_t target_y = height - source_y - 1;
      output.cells[static_cast<size_t>(target_y * width + x)] = cell;
    }
  }
  if (!output.valid()) {
    if (error) *error = "loaded map is invalid: " + yaml_path;
    return false;
  }
  *map = std::move(output);
  return true;
}

double map_match_score(
  const GridMap & map, const ScanData & scan, const Pose2D & pose,
  const MatchConfig & config)
{
  if (!map.valid()) return 0.0;
  const auto points = endpoints(scan, pose, config.max_beams);
  if (points.empty()) return 0.0;
  const double co = std::cos(map.origin.yaw);
  const double so = std::sin(map.origin.yaw);
  size_t hits = 0;
  size_t evaluated = 0;
  for (const auto & point : points) {
    const double dx = point.first - map.origin.x;
    const double dy = point.second - map.origin.y;
    const double local_x = co * dx + so * dy;
    const double local_y = -so * dx + co * dy;
    const int x = static_cast<int>(std::floor(local_x / map.resolution));
    const int y = static_cast<int>(std::floor(local_y / map.resolution));
    if (x < 0 || y < 0 || x >= static_cast<int>(map.width) || y >= static_cast<int>(map.height)) {
      ++evaluated;
      continue;
    }
    const int8_t cell = map.cells[static_cast<size_t>(y) * map.width + x];
    if (cell < 0) continue;
    ++evaluated;
    if (cell >= config.occupied_threshold) ++hits;
  }
  return evaluated ? static_cast<double>(hits) / evaluated : 0.0;
}

double scan_match_score(
  const ScanData & current_scan, const Pose2D & current_pose,
  const ScanRecord & reference, const MatchConfig & config)
{
  const auto reference_points = endpoints(reference.scan, reference.pose, config.max_beams);
  const auto current_points = endpoints(current_scan, current_pose, config.max_beams);
  if (reference_points.empty() || current_points.empty() || config.correlation_resolution <= 0.0) {
    return 0.0;
  }
  std::unordered_set<int64_t> bins;
  for (const auto & point : reference_points) {
    bins.insert(bin_key(
      static_cast<int32_t>(std::floor(point.first / config.correlation_resolution)),
      static_cast<int32_t>(std::floor(point.second / config.correlation_resolution))));
  }
  const int radius = std::max(
    0, static_cast<int>(std::ceil(config.correlation_distance / config.correlation_resolution)));
  size_t hits = 0;
  for (const auto & point : current_points) {
    const int32_t bx = static_cast<int32_t>(std::floor(point.first / config.correlation_resolution));
    const int32_t by = static_cast<int32_t>(std::floor(point.second / config.correlation_resolution));
    bool found = false;
    for (int dx = -radius; dx <= radius && !found; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        if (bins.count(bin_key(bx + dx, by + dy))) {
          found = true;
          break;
        }
      }
    }
    if (found) ++hits;
  }
  return static_cast<double>(hits) / current_points.size();
}

MatchResult search_pose(
  const GridMap & map, const ScanData & scan, const Pose2D & seed,
  const ScanRecord * reference, const MatchConfig & config)
{
  MatchResult coarse = search_window(
    map, scan, seed, config.search_xy, config.search_yaw,
    config.coarse_xy_step, config.coarse_yaw_step, reference, config);
  if (!coarse.valid) return coarse;
  MatchResult refined = search_window(
    map, scan, coarse.pose, config.coarse_xy_step, config.coarse_yaw_step,
    config.refine_xy_step, config.refine_yaw_step, reference, config);
  return refined.valid ? refined : coarse;
}

}  // namespace relocalization
