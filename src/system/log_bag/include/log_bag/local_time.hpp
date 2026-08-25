#pragma once

#include <cctype>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace log_bag {

constexpr std::time_t kCstOffsetSeconds = 8 * 60 * 60;

inline bool cst_tm(std::time_t value, std::tm & out) {
  const std::time_t shifted = value + kCstOffsetSeconds;
  return ::gmtime_r(&shifted, &out) != nullptr;
}

inline std::string cst_iso8601(std::time_t value) {
  std::tm cst {};
  if (!cst_tm(value, cst)) {
    return {};
  }
  std::ostringstream out;
  out << std::put_time(&cst, "%Y-%m-%dT%H:%M:%S") << "+08:00";
  return out.str();
}

inline std::string cst_filename_timestamp(std::time_t value) {
  std::tm cst {};
  if (!cst_tm(value, cst)) {
    return {};
  }
  std::ostringstream out;
  out << std::put_time(&cst, "%Y%m%dT%H%M%S") << "+0800";
  return out.str();
}

inline bool compact_datetime_prefix_valid(const std::string & compact) {
  if (compact.size() < 15 || compact[8] != 'T') {
    return false;
  }
  for (std::size_t i = 0; i < 15; ++i) {
    if (i == 8) {
      continue;
    }
    if (!std::isdigit(static_cast<unsigned char>(compact[i]))) {
      return false;
    }
  }
  return true;
}

inline std::string cst_iso8601_from_compact(const std::string & compact) {
  if (!compact_datetime_prefix_valid(compact)) {
    return {};
  }
  std::tm wall_tm {};
  std::istringstream input(compact.substr(0, 15));
  input >> std::get_time(&wall_tm, "%Y%m%dT%H%M%S");
  if (input.fail()) {
    return {};
  }
  std::time_t epoch = ::timegm(&wall_tm);

  // Convert recorder names carrying any historical numeric offset to CST.
  if (compact.size() >= 20 && (compact[15] == '+' || compact[15] == '-') &&
    std::isdigit(static_cast<unsigned char>(compact[16])) &&
    std::isdigit(static_cast<unsigned char>(compact[17])) &&
    std::isdigit(static_cast<unsigned char>(compact[18])) &&
    std::isdigit(static_cast<unsigned char>(compact[19])))
  {
    const int hours = std::stoi(compact.substr(16, 2));
    const int minutes = std::stoi(compact.substr(18, 2));
    const int sign = compact[15] == '-' ? -1 : 1;
    epoch -= sign * (hours * 60 + minutes) * 60;
    return cst_iso8601(epoch);
  }

  // Legacy offset-free names were UTC.
  return cst_iso8601(epoch);
}

}  // namespace log_bag
