#ifndef MANAGER__SEMANTIC_LOCATION_QUERY_HPP_
#define MANAGER__SEMANTIC_LOCATION_QUERY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace manager {

struct SemanticRegion {
  std::string name;
  double min_x;
  double min_y;
  double max_x;
  double max_y;
};

struct SemanticDistance {
  std::string name;
  double distance;
};

// Read-only semantic-map lookup. The caller supplies the latest localized x/y and
// semantic regions from its map loader; this class has no ROS side effects.
class SemanticLocationQuery {
public:
  explicit SemanticLocationQuery(std::vector<SemanticRegion> regions = {})
  : regions_(std::move(regions)) {}

  void set_regions(std::vector<SemanticRegion> regions) {
    regions_ = std::move(regions);
  }

  const std::vector<SemanticRegion> & regions() const {
    return regions_;
  }

  std::string query_current_location(double x, double y) const {
    for (const auto & region : regions_) {
      if (x >= region.min_x && x <= region.max_x &&
        y >= region.min_y && y <= region.max_y)
      {
        return region.name;
      }
    }
    return "unknown";
  }

  double distance_to(const std::string & name, double x, double y) const {
    for (const auto & region : regions_) {
      if (region.name == name) {
        return distance_to_region(region, x, y);
      }
    }
    return -1.0;
  }

  std::vector<SemanticDistance> nearest(
    double x, double y, std::size_t limit,
    const std::string & exclude_name = "") const {
    std::vector<SemanticDistance> result;
    for (const auto & region : regions_) {
      if (!exclude_name.empty() && region.name == exclude_name) {
        continue;
      }
      result.push_back({region.name, distance_to_region(region, x, y)});
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
  static double distance_to_region(const SemanticRegion & region, double x, double y) {
    const double dx = std::max({region.min_x - x, 0.0, x - region.max_x});
    const double dy = std::max({region.min_y - y, 0.0, y - region.max_y});
    return std::hypot(dx, dy);
  }

  std::vector<SemanticRegion> regions_;
};

}  // namespace manager

#endif
