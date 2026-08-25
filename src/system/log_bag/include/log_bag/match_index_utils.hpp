#pragma once

#include <cctype>
#include <cstddef>
#include <string>
#include <vector>

namespace log_bag {

struct MatchBagEntry {
  std::string path;
  std::string raw;
  bool tagged{false};
};

inline std::string json_trim(const std::string & value) {
  std::size_t begin = 0;
  while (begin < value.size() &&
    std::isspace(static_cast<unsigned char>(value[begin])))
  {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin &&
    std::isspace(static_cast<unsigned char>(value[end - 1])))
  {
    --end;
  }
  return value.substr(begin, end - begin);
}

inline std::vector<std::string> split_json_object_members(const std::string & value) {
  std::vector<std::string> members;
  std::size_t start = 0;
  int object_depth = 0;
  int array_depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (std::size_t i = 0; i < value.size(); ++i) {
    const char ch = value[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (in_string && ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (ch == '{') {
      ++object_depth;
    } else if (ch == '}') {
      --object_depth;
    } else if (ch == '[') {
      ++array_depth;
    } else if (ch == ']') {
      --array_depth;
    } else if (ch == ',' && object_depth == 0 && array_depth == 0) {
      const std::string member = json_trim(value.substr(start, i - start));
      if (!member.empty()) {
        members.push_back(member);
      }
      start = i + 1;
    }
  }
  const std::string member = json_trim(value.substr(start));
  if (!member.empty()) {
    members.push_back(member);
  }
  return members;
}

inline std::string parse_json_key(const std::string & member) {
  std::size_t i = 0;
  while (i < member.size() &&
    std::isspace(static_cast<unsigned char>(member[i])))
  {
    ++i;
  }
  if (i >= member.size() || member[i] != '"') {
    return {};
  }
  ++i;
  std::string key;
  bool escaped = false;
  for (; i < member.size(); ++i) {
    const char ch = member[i];
    if (escaped) {
      switch (ch) {
        case 'n':
          key.push_back('\n');
          break;
        case 'r':
          key.push_back('\r');
          break;
        case 't':
          key.push_back('\t');
          break;
        default:
          key.push_back(ch);
          break;
      }
      escaped = false;
    } else if (ch == '\\') {
      escaped = true;
    } else if (ch == '"') {
      return key;
    } else {
      key.push_back(ch);
    }
  }
  return {};
}

inline bool json_member_has_tags(const std::string & member) {
  const std::size_t key = member.find("\"tags\"");
  if (key == std::string::npos) {
    return false;
  }
  const std::size_t open = member.find('[', key + 6);
  if (open == std::string::npos) {
    return false;
  }
  bool in_string = false;
  bool escaped = false;
  int depth = 0;
  for (std::size_t i = open; i < member.size(); ++i) {
    const char ch = member[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (in_string && ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (ch == '[') {
      ++depth;
    } else if (ch == ']') {
      --depth;
      if (depth == 0) {
        return !json_trim(member.substr(open + 1, i - open - 1)).empty();
      }
    }
  }
  return false;
}

inline std::vector<MatchBagEntry> parse_match_bag_members(const std::string & value) {
  std::vector<MatchBagEntry> entries;
  for (const auto & raw : split_json_object_members(value)) {
    entries.push_back({parse_json_key(raw), raw, json_member_has_tags(raw)});
  }
  return entries;
}

}  // namespace log_bag
