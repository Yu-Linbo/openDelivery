#include <iostream>

#include "manager/semantic_location_query.hpp"

int main() {
  manager::SemanticLocationQuery query({
    {"loading_area", -14.0, 12.0, -12.0, 14.0},
    {"charging_area", -12.0, 12.0, -10.0, 14.0},
  });
  // The caller supplies its latest localized x/y; no ROS node is started here.
  std::cout << query.query_current_location(-13.703, 12.825) << std::endl;
  return 0;
}
