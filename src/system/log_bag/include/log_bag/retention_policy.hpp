#pragma once

#include <cstddef>
#include <cstdint>
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

// Sizes must be ordered oldest to newest. A hard storage cleanup deliberately
// overrides task tags and removes the oldest bags until the hysteresis target
// is reached, so recording does not immediately retrigger cleanup.
inline std::vector<bool> storage_keep_mask(
  const std::vector<std::uintmax_t> & sizes, std::uintmax_t target_bytes)
{
  std::vector<bool> keep(sizes.size(), true);
  std::uintmax_t total = 0;
  for (const auto size : sizes) {
    total += size;
  }
  for (std::size_t i = 0; i < sizes.size() && total > target_bytes; ++i) {
    keep[i] = false;
    total -= sizes[i];
  }
  return keep;
}

}  // namespace log_bag
