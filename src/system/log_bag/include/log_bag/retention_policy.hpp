#pragma once

#include <cstddef>
#include <vector>

namespace log_bag {

// Keep every tagged bag and the directly adjacent untagged bag on each side of
// a tagged run. Before the first task exists, retain only the newest untagged
// backup as the rolling pre-task bag.
inline std::vector<bool> retention_keep_mask(const std::vector<bool> & tagged) {
  std::vector<bool> keep(tagged.size(), false);
  bool has_tagged = false;
  for (std::size_t i = 0; i < tagged.size(); ++i) {
    if (!tagged[i]) {
      continue;
    }
    has_tagged = true;
    keep[i] = true;
    if (i > 0 && !tagged[i - 1]) {
      keep[i - 1] = true;
    }
    if (i + 1 < tagged.size() && !tagged[i + 1]) {
      keep[i + 1] = true;
    }
  }
  if (!has_tagged && !tagged.empty()) {
    keep.back() = true;
  }
  return keep;
}

}  // namespace log_bag
