#include "log_bag/local_time.hpp"
#include "log_bag/match_index_utils.hpp"
#include "log_bag/retention_policy.hpp"

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

int main() {
  using log_bag::retention_keep_mask;

  assert((retention_keep_mask({}) == std::vector<bool>{}));
  assert((retention_keep_mask({false, false, false}) ==
    std::vector<bool>{false, false, true}));
  assert((retention_keep_mask({false, false, true, true, false, false}) ==
    std::vector<bool>{false, true, true, true, true, false}));
  assert((retention_keep_mask({true, false, false, true}) ==
    std::vector<bool>{true, true, true, true}));
  assert((retention_keep_mask({false, true, false, false, false}) ==
    std::vector<bool>{true, true, true, false, false}));

  const std::string members =
    "\"/bags/001_terminal_bag\": {\"tags\": [], \"topics\": [\"/a\", \"/b\"]},"
    "\"/bags/002_terminal_bag\": {\"tags\": [1001]},"
    "\"/bags/003_terminal_bag\": {\"tags\": [\"task-2\", \"task-3\"]}";
  const auto parsed = log_bag::parse_match_bag_members(members);
  assert(parsed.size() == 3);
  assert(parsed[0].path == "/bags/001_terminal_bag");
  assert(!parsed[0].tagged);
  assert(parsed[1].path == "/bags/002_terminal_bag");
  assert(parsed[1].tagged);
  assert(parsed[2].path == "/bags/003_terminal_bag");
  assert(parsed[2].tagged);

  const auto escaped = log_bag::parse_match_bag_members(
    "\"/bags/a\\\\b_terminal_bag\": {\"tags\": []}");
  assert(escaped.size() == 1);
  assert(escaped[0].path == "/bags/a\\b_terminal_bag");

  ::setenv("TZ", "America/Asuncion", 1);
  ::tzset();
  std::tm utc {};
  utc.tm_year = 2026 - 1900;
  utc.tm_mon = 7;
  utc.tm_mday = 25;
  utc.tm_hour = 5;
  utc.tm_min = 32;
  utc.tm_sec = 34;
  const std::time_t sample = ::timegm(&utc);
  assert(log_bag::cst_iso8601(sample) == "2026-08-25T13:32:34+08:00");
  assert(log_bag::cst_filename_timestamp(sample) == "20260825T133234+0800");
  assert(log_bag::cst_iso8601_from_compact("20260825T023234-0300") ==
    "2026-08-25T13:32:34+08:00");
  assert(log_bag::cst_iso8601_from_compact("20260825T023234-0300_001") ==
    "2026-08-25T13:32:34+08:00");
  assert(log_bag::cst_iso8601_from_compact("20260825T133234+0800") ==
    "2026-08-25T13:32:34+08:00");
  assert(log_bag::cst_iso8601_from_compact("20260825T053234") ==
    "2026-08-25T13:32:34+08:00");
  return 0;
}
