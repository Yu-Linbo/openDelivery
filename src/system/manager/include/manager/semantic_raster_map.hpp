#ifndef MANAGER__SEMANTIC_RASTER_MAP_HPP_
#define MANAGER__SEMANTIC_RASTER_MAP_HPP_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <json/json.h>
#include <opencv2/imgcodecs.hpp>

#include "manager/semantic_location_query.hpp"

namespace manager {

class SemanticRasterMap {
public:
  bool load(const std::string & yaml_path, std::string * error) {
    clear();
    std::ifstream yaml(yaml_path);
    if (!yaml) {
      return fail("semantic yaml not found: " + yaml_path, error);
    }

    std::string image_name;
    std::string legend_name;
    std::string line;
    while (std::getline(yaml, line)) {
      const auto comment = line.find('#');
      if (comment != std::string::npos) {
        line.erase(comment);
      }
      const auto colon = line.find(':');
      if (colon == std::string::npos) {
        continue;
      }
      const std::string key = trim(line.substr(0, colon));
      const std::string value = unquote(trim(line.substr(colon + 1)));
      if (key == "image") {
        image_name = value;
      } else if (key == "legend") {
        legend_name = value;
      } else if (key == "resolution") {
        try {
          resolution_ = std::stod(value);
        } catch (const std::exception &) {
          return fail("invalid semantic resolution in " + yaml_path, error);
        }
      } else if (key == "origin") {
        if (!parse_origin(value, &origin_x_, &origin_y_, &origin_yaw_)) {
          return fail("invalid semantic origin in " + yaml_path, error);
        }
      }
    }
    if (image_name.empty() || legend_name.empty() || resolution_ <= 0.0) {
      return fail("semantic yaml needs image, legend and positive resolution: " + yaml_path, error);
    }

    const std::string base = parent_path(yaml_path);
    const std::string image_path = join_path(base, image_name);
    const std::string legend_path = join_path(base, legend_name);
    image_ = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image_.empty()) {
      return fail("semantic image cannot be read: " + image_path, error);
    }
    if (!load_legend(legend_path, error)) {
      clear();
      return false;
    }
    index_pixels();
    if (labels_.empty()) {
      return fail("semantic legend has no location labels: " + legend_path, error);
    }
    yaml_path_ = yaml_path;
    valid_ = true;
    return true;
  }

  void clear() {
    valid_ = false;
    yaml_path_.clear();
    image_.release();
    resolution_ = 0.0;
    origin_x_ = 0.0;
    origin_y_ = 0.0;
    origin_yaw_ = 0.0;
    labels_.clear();
    label_index_by_color_.clear();
  }

  bool valid() const {return valid_;}
  const std::string & yaml_path() const {return yaml_path_;}
  std::size_t label_count() const {return labels_.size();}

  std::string query_current_location(double x, double y) const {
    int col = 0;
    int row = 0;
    if (!world_to_pixel(x, y, &col, &row)) {
      return "unknown";
    }
    const cv::Vec3b bgr = image_.at<cv::Vec3b>(row, col);
    const auto it = label_index_by_color_.find(color_key(bgr[2], bgr[1], bgr[0]));
    if (it == label_index_by_color_.end()) {
      return "unknown";
    }
    return labels_[it->second].id;
  }

  double distance_to(const std::string & name, double x, double y) const {
    const Label * label = find_label(name);
    if (!label || label->world_points.empty()) {
      return -1.0;
    }
    const std::string current = query_current_location(x, y);
    if (current == label->id) {
      return 0.0;
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto & point : label->world_points) {
      best = std::min(best, std::hypot(point.first - x, point.second - y));
    }
    return std::isfinite(best) ? best : -1.0;
  }

  std::vector<SemanticDistance> nearest(
    double x, double y, std::size_t limit,
    const std::string & exclude_name = "") const
  {
    std::vector<SemanticDistance> result;
    for (const auto & label : labels_) {
      if (label.id == exclude_name || label.name == exclude_name || label.world_points.empty()) {
        continue;
      }
      const double distance = distance_to(label.id, x, y);
      if (distance >= 0.0) {
        result.push_back({label.id, distance});
      }
    }
    std::sort(result.begin(), result.end(), [](const auto & a, const auto & b) {
      return a.distance < b.distance;
    });
    if (result.size() > limit) {
      result.resize(limit);
    }
    return result;
  }

private:
  struct Label {
    std::string id;
    std::string name;
    int color{0};
    std::vector<std::pair<double, double>> world_points;
  };

  static std::string trim(const std::string & value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
  }

  static std::string unquote(const std::string & value) {
    if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
      (value.front() == '\'' && value.back() == '\'')))
    {
      return value.substr(1, value.size() - 2);
    }
    return value;
  }

  static bool parse_origin(
    std::string value, double * x, double * y, double * yaw)
  {
    value.erase(std::remove(value.begin(), value.end(), '['), value.end());
    value.erase(std::remove(value.begin(), value.end(), ']'), value.end());
    std::replace(value.begin(), value.end(), ',', ' ');
    std::stringstream stream(value);
    return static_cast<bool>(stream >> *x >> *y >> *yaw);
  }

  static std::string parent_path(const std::string & path) {
    const auto slash = path.find_last_of("/\\");
    return slash == std::string::npos ? "." : path.substr(0, slash);
  }

  static std::string join_path(const std::string & base, const std::string & child) {
    if (child.empty() || child.front() == '/') {
      return child;
    }
    return base.empty() || base.back() == '/' ? base + child : base + "/" + child;
  }

  static int color_key(int red, int green, int blue) {
    return ((red & 0xff) << 16) | ((green & 0xff) << 8) | (blue & 0xff);
  }

  static bool parse_color(const std::string & value, int * result) {
    if (value.size() != 7 || value.front() != '#') {
      return false;
    }
    try {
      *result = std::stoi(value.substr(1), nullptr, 16);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  static bool is_location_label(const std::string & id) {
    return !id.empty() && id != "background" && id != "obstacle";
  }

  bool load_legend(const std::string & path, std::string * error) {
    std::ifstream input(path);
    if (!input) {
      return fail("semantic legend not found: " + path, error);
    }
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string parse_error;
    if (!Json::parseFromStream(builder, input, &root, &parse_error)) {
      return fail("invalid semantic legend " + path + ": " + parse_error, error);
    }
    const Json::Value values = root["labels"];
    if (!values.isArray()) {
      return fail("semantic legend labels must be an array: " + path, error);
    }
    for (const auto & value : values) {
      const std::string id = value.get("id", "").asString();
      if (!is_location_label(id)) {
        continue;
      }
      int color = 0;
      if (!parse_color(value.get("color", "").asString(), &color)) {
        continue;
      }
      Label label;
      label.id = id;
      label.name = value.get("name", id).asString();
      label.color = color;
      label_index_by_color_[color] = labels_.size();
      labels_.push_back(std::move(label));
    }
    return true;
  }

  void index_pixels() {
    const double cosine = std::cos(origin_yaw_);
    const double sine = std::sin(origin_yaw_);
    for (int row = 0; row < image_.rows; ++row) {
      for (int col = 0; col < image_.cols; ++col) {
        const cv::Vec3b bgr = image_.at<cv::Vec3b>(row, col);
        const auto found = label_index_by_color_.find(color_key(bgr[2], bgr[1], bgr[0]));
        if (found == label_index_by_color_.end()) {
          continue;
        }
        const double local_x = (static_cast<double>(col) + 0.5) * resolution_;
        const double local_y =
          (static_cast<double>(image_.rows - 1 - row) + 0.5) * resolution_;
        labels_[found->second].world_points.emplace_back(
          origin_x_ + cosine * local_x - sine * local_y,
          origin_y_ + sine * local_x + cosine * local_y);
      }
    }
  }

  bool world_to_pixel(double x, double y, int * col, int * row) const {
    if (!valid_ || image_.empty()) {
      return false;
    }
    const double dx = x - origin_x_;
    const double dy = y - origin_y_;
    const double cosine = std::cos(origin_yaw_);
    const double sine = std::sin(origin_yaw_);
    const double local_x = cosine * dx + sine * dy;
    const double local_y = -sine * dx + cosine * dy;
    const int map_col = static_cast<int>(std::floor(local_x / resolution_));
    const int map_row_from_bottom = static_cast<int>(std::floor(local_y / resolution_));
    const int image_row = image_.rows - 1 - map_row_from_bottom;
    if (map_col < 0 || map_col >= image_.cols || image_row < 0 || image_row >= image_.rows) {
      return false;
    }
    *col = map_col;
    *row = image_row;
    return true;
  }

  const Label * find_label(const std::string & name) const {
    for (const auto & label : labels_) {
      if (label.id == name || label.name == name) {
        return &label;
      }
    }
    return nullptr;
  }

  static bool fail(const std::string & message, std::string * error) {
    if (error) {
      *error = message;
    }
    return false;
  }

  bool valid_{false};
  std::string yaml_path_;
  cv::Mat image_;
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  std::vector<Label> labels_;
  std::unordered_map<int, std::size_t> label_index_by_color_;
};

}  // namespace manager

#endif
