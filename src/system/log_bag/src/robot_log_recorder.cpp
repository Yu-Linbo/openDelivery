#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <custom_msgs_srvs/msg/task_status.hpp>
#include <custom_msgs_srvs/msg/robot_status.hpp>
#include <sqlite3.h>

#include "log_bag/local_time.hpp"
#include "log_bag/match_index_utils.hpp"
#include "log_bag/recording_topics.hpp"
#include "log_bag/retention_policy.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <limits.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr std::uintmax_t kDefaultMaxBagBytes = 50U * 1024U * 1024U;
constexpr std::uintmax_t kDefaultMaxRobotBytes = 1024U * 1024U * 1024U;
constexpr std::uintmax_t kDefaultPruneTargetBytes = 500U * 1024U * 1024U;
constexpr std::uintmax_t kMinArchiveBagBytes = 64U * 1024U;
constexpr auto kCriticalTopicGrace = std::chrono::seconds(10);
constexpr auto kCriticalTopicCheckInterval = std::chrono::seconds(10);
constexpr auto kCriticalTopicStall = std::chrono::seconds(30);
constexpr auto kRobotStatusStall = std::chrono::seconds(30);
constexpr auto kMaxRecorderRestartBackoff = std::chrono::seconds(60);
constexpr auto kStorageCheckInterval = std::chrono::seconds(5);

std::atomic_bool g_stop_requested{false};

void on_signal(int) {
  g_stop_requested.store(true);
}

std::string trim(const std::string & s) {
  const char * ws = " \t\r\n";
  const auto start = s.find_first_not_of(ws);
  if (start == std::string::npos) {
    return {};
  }
  const auto end = s.find_last_not_of(ws);
  return s.substr(start, end - start + 1);
}

std::string shell_quote(const std::string & value) {
  std::string out = "'";
  for (char ch : value) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out += "'";
  return out;
}

std::string json_escape(const std::string & s) {
  std::ostringstream os;
  for (char ch : s) {
    switch (ch) {
      case '\\':
        os << "\\\\";
        break;
      case '"':
        os << "\\\"";
        break;
      case '\n':
        os << "\\n";
        break;
      case '\r':
        os << "\\r";
        break;
      case '\t':
        os << "\\t";
        break;
      default:
        os << ch;
        break;
    }
  }
  return os.str();
}

bool path_exists(const std::string & path) {
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0;
}

bool is_directory(const std::string & path) {
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool is_symlink(const std::string & path) {
  struct stat st {};
  return ::lstat(path.c_str(), &st) == 0 && S_ISLNK(st.st_mode);
}

bool is_regular_file(const std::string & path) {
  struct stat st {};
  return ::lstat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

bool remove_path(const std::string & path) {
  if (!path_exists(path)) {
    return true;
  }
  return ::unlink(path.c_str()) == 0;
}

bool create_symlink(const std::string & target, const std::string & link_path) {
  remove_path(link_path);
  return ::symlink(target.c_str(), link_path.c_str()) == 0;
}

std::string join_path(const std::string & a, const std::string & b) {
  if (a.empty()) {
    return b;
  }
  if (a.back() == '/') {
    return a + b;
  }
  return a + "/" + b;
}

void ensure_dir(const std::string & path) {
  if (path.empty() || is_directory(path)) {
    return;
  }
  std::string current;
  if (path.front() == '/') {
    current = "/";
  }
  std::stringstream ss(path);
  std::string part;
  while (std::getline(ss, part, '/')) {
    if (part.empty()) {
      continue;
    }
    if (!current.empty() && current.back() != '/') {
      current += "/";
    }
    current += part;
    if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
      throw std::runtime_error("mkdir failed for " + current + ": " + std::strerror(errno));
    }
  }
}

std::string dirname_of(const std::string & path) {
  const auto pos = path.find_last_of('/');
  if (pos == std::string::npos) {
    return ".";
  }
  if (pos == 0) {
    return "/";
  }
  return path.substr(0, pos);
}

std::string basename_of(const std::string & path) {
  const auto pos = path.find_last_of('/');
  if (pos == std::string::npos) {
    return path;
  }
  return path.substr(pos + 1);
}

bool same_existing_path(const std::string & a, const std::string & b) {
  char resolved_a[PATH_MAX] {};
  char resolved_b[PATH_MAX] {};
  if (::realpath(a.c_str(), resolved_a) == nullptr ||
    ::realpath(b.c_str(), resolved_b) == nullptr)
  {
    return false;
  }
  return std::strcmp(resolved_a, resolved_b) == 0;
}

std::string now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const auto value = log_bag::cst_iso8601(
    std::chrono::system_clock::to_time_t(now));
  return value.empty() ? "1970-01-01T00:00:00+00:00" : value;
}

std::string timestamp_for_filename() {
  const auto now = std::chrono::system_clock::now();
  const auto value = log_bag::cst_filename_timestamp(
    std::chrono::system_clock::to_time_t(now));
  return value.empty() ? "19700101T000000+0000" : value;
}

std::uintmax_t directory_size(const std::string & path) {
  struct stat st {};
  if (::lstat(path.c_str(), &st) != 0) {
    return 0;
  }
  if (S_ISREG(st.st_mode)) {
    return static_cast<std::uintmax_t>(st.st_size);
  }
  if (!S_ISDIR(st.st_mode)) {
    return 0;
  }
  std::uintmax_t total = 0;
  DIR * dir = ::opendir(path.c_str());
  if (!dir) {
    return 0;
  }
  while (dirent * ent = ::readdir(dir)) {
    const std::string name = ent->d_name;
    if (name == "." || name == "..") {
      continue;
    }
    total += directory_size(join_path(path, name));
  }
  ::closedir(dir);
  return total;
}

bool move_path(const std::string & from, const std::string & to) {
  ensure_dir(dirname_of(to));
  if (!path_exists(from)) {
    return false;
  }
  return ::rename(from.c_str(), to.c_str()) == 0;
}

constexpr const char * kTaskTagsMarker = ".opendelivery_task_tags";
constexpr const char * kRobotStatusSidecar = ".opendelivery_robot_status.json";

struct RecordedRobotStatus {
  std::int64_t timestamp_ns{0};
  std::string robot_name;
  std::string robot_model;
  std::string current_map;
  std::string current_position;
  std::string robot_status;
  std::string task_status;
  std::string control_status;
  std::string localization_method;
  bool is_simulation{false};
  float task_progress{-1.0F};
};

std::int64_t system_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

RecordedRobotStatus record_robot_status(
  const custom_msgs_srvs::msg::RobotStatus & msg)
{
  RecordedRobotStatus out;
  out.timestamp_ns = system_now_ns();
  out.robot_name = msg.robot_name;
  out.robot_model = msg.robot_model;
  out.current_map = msg.current_map;
  out.current_position = msg.current_position;
  out.robot_status = msg.robot_status;
  out.task_status = msg.task_status;
  out.control_status = msg.control_status;
  out.localization_method = msg.localization_method;
  out.is_simulation = msg.is_simulation;
  out.task_progress = std::isfinite(msg.task_progress) ? msg.task_progress : -1.0F;
  return out;
}

void write_robot_status_sidecar(
  const std::string & bag_path, const std::vector<RecordedRobotStatus> & samples)
{
  if (!is_directory(bag_path) || samples.empty()) {
    return;
  }
  const std::string path = join_path(bag_path, kRobotStatusSidecar);
  const std::string tmp = path + ".tmp";
  std::ofstream out(tmp, std::ios::trunc);
  if (!out) {
    return;
  }
  out << "{\n  \"version\": 1,\n  \"statuses\": [";
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const auto & row = samples[index];
    out << (index == 0 ? "\n" : ",\n")
        << "    {\"timestamp_ns\": " << row.timestamp_ns
        << ", \"robot_name\": \"" << json_escape(row.robot_name)
        << "\", \"robot_model\": \"" << json_escape(row.robot_model)
        << "\", \"current_map\": \"" << json_escape(row.current_map)
        << "\", \"current_position\": \"" << json_escape(row.current_position)
        << "\", \"robot_status\": \"" << json_escape(row.robot_status)
        << "\", \"task_status\": \"" << json_escape(row.task_status)
        << "\", \"control_status\": \"" << json_escape(row.control_status)
        << "\", \"localization_method\": \"" << json_escape(row.localization_method)
        << "\", \"is_simulation\": " << (row.is_simulation ? "true" : "false")
        << ", \"task_progress\": " << std::setprecision(9) << row.task_progress << "}";
  }
  out << "\n  ]\n}\n";
  out.close();
  ::rename(tmp.c_str(), path.c_str());
}

void write_bag_tags_marker(
  const std::string & bag_path, const std::vector<std::string> & tags)
{
  if (!is_directory(bag_path) || tags.empty()) {
    return;
  }
  const std::string marker = join_path(bag_path, kTaskTagsMarker);
  const std::string tmp = marker + ".tmp";
  std::ofstream out(tmp, std::ios::trunc);
  if (!out) {
    return;
  }
  for (const auto & tag : tags) {
    out << std::quoted(tag) << "\n";
  }
  out.close();
  ::rename(tmp.c_str(), marker.c_str());
}

std::vector<std::string> read_bag_tags_marker(const std::string & bag_path) {
  std::vector<std::string> tags;
  std::ifstream in(join_path(bag_path, kTaskTagsMarker));
  std::string tag;
  while (in >> std::quoted(tag)) {
    if (!trim(tag).empty() && std::find(tags.begin(), tags.end(), tag) == tags.end()) {
      tags.push_back(tag);
    }
  }
  return tags;
}

bool has_suffix(const std::string & s, const std::string & suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::vector<std::string> list_dir_basenames(const std::string & path) {
  std::vector<std::string> names;
  DIR * dir = ::opendir(path.c_str());
  if (!dir) {
    return names;
  }
  while (dirent * ent = ::readdir(dir)) {
    const std::string name = ent->d_name;
    if (name == "." || name == "..") {
      continue;
    }
    names.push_back(name);
  }
  ::closedir(dir);
  return names;
}

std::uintmax_t sqlite_pragma_value(sqlite3 * db, const char * statement) {
  sqlite3_stmt * query = nullptr;
  if (sqlite3_prepare_v2(db, statement, -1, &query, nullptr) != SQLITE_OK) {
    return 0;
  }
  const auto value = sqlite3_step(query) == SQLITE_ROW
    ? static_cast<std::uintmax_t>(sqlite3_column_int64(query, 0)) : 0;
  sqlite3_finalize(query);
  return value;
}

std::uintmax_t sqlite_logical_bag_size(const std::string & path) {
  std::uintmax_t total = 0;
  for (const auto & name : list_dir_basenames(path)) {
    if (!has_suffix(name, ".db3")) {
      continue;
    }
    sqlite3 * db = nullptr;
    const std::string database = join_path(path, name);
    if (sqlite3_open_v2(
        database.c_str(), &db, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr) != SQLITE_OK)
    {
      if (db) {
        sqlite3_close(db);
      }
      return 0;
    }
    sqlite3_busy_timeout(db, 100);
    const auto page_count = sqlite_pragma_value(db, "PRAGMA page_count");
    const auto page_size = sqlite_pragma_value(db, "PRAGMA page_size");
    sqlite3_close(db);
    if (!page_count || !page_size) {
      return 0;
    }
    total += page_count * page_size;
  }
  return total;
}

bool sqlite_topic_message_count(
  const std::string & path, const std::string & topic, std::uintmax_t * total)
{
  if (!total) {
    return false;
  }
  *total = 0;
  bool found_database = false;
  for (const auto & name : list_dir_basenames(path)) {
    if (!has_suffix(name, ".db3")) {
      continue;
    }
    found_database = true;
    sqlite3 * db = nullptr;
    const std::string database = join_path(path, name);
    if (sqlite3_open_v2(
        database.c_str(), &db, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr) != SQLITE_OK)
    {
      if (db) {
        sqlite3_close(db);
      }
      return false;
    }
    sqlite3_busy_timeout(db, 100);
    sqlite3_stmt * query = nullptr;
    constexpr const char * statement =
      "SELECT COUNT(m.id) FROM topics t LEFT JOIN messages m ON m.topic_id=t.id "
      "WHERE t.name=?";
    if (sqlite3_prepare_v2(db, statement, -1, &query, nullptr) != SQLITE_OK) {
      sqlite3_close(db);
      return false;
    }
    sqlite3_bind_text(query, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(query) != SQLITE_ROW) {
      sqlite3_finalize(query);
      sqlite3_close(db);
      return false;
    }
    *total += static_cast<std::uintmax_t>(sqlite3_column_int64(query, 0));
    sqlite3_finalize(query);
    sqlite3_close(db);
  }
  return found_database;
}

std::string find_custom_msgs_prefix(const std::string & workspace_root) {
  const std::string resource =
    "share/ament_index/resource_index/packages/custom_msgs_srvs";
  const char * raw_prefixes = std::getenv("AMENT_PREFIX_PATH");
  std::stringstream prefixes(raw_prefixes ? raw_prefixes : "");
  std::string prefix;
  while (std::getline(prefixes, prefix, ':')) {
    if (!prefix.empty() && path_exists(join_path(prefix, resource))) {
      return prefix;
    }
  }
  for (const auto & candidate : {
      join_path(join_path(workspace_root, "install"), "custom_msgs_srvs"),
      join_path(workspace_root, "install")})
  {
    if (path_exists(join_path(candidate, resource))) {
      return candidate;
    }
  }
  return {};
}

std::string file_prefix_before_terminal(const std::string & name) {
  const std::string marker = "_terminal";
  const auto pos = name.find(marker);
  if (pos == std::string::npos) {
    return {};
  }
  return name.substr(0, pos);
}

std::string iso8601_from_compact_timestamp(const std::string & compact) {
  const auto value = log_bag::cst_iso8601_from_compact(compact);
  return value.empty() ? now_iso8601() : value;
}

std::string file_mtime_iso8601(const std::string & path) {
  struct stat st {};
  if (::stat(path.c_str(), &st) != 0) {
    return now_iso8601();
  }
  const auto value = log_bag::cst_iso8601(st.st_mtime);
  return value.empty() ? now_iso8601() : value;
}

pid_t spawn_shell_command(const std::string & command, int log_fd) {
  pid_t pid = ::fork();
  if (pid < 0) {
    throw std::runtime_error("fork failed: " + std::string(std::strerror(errno)));
  }
  if (pid == 0) {
    ::setpgid(0, 0);
    if (log_fd >= 0) {
      ::dup2(log_fd, STDOUT_FILENO);
      ::dup2(log_fd, STDERR_FILENO);
    }
    execl("/bin/bash", "bash", "-lc", command.c_str(), static_cast<char *>(nullptr));
    _exit(127);
  }
  ::setpgid(pid, pid);
  return pid;
}

bool process_exited(pid_t pid, int * status) {
  return ::waitpid(pid, status, WNOHANG) == pid;
}

void stop_process_group(pid_t pid, int signal_number = SIGINT) {
  if (pid > 0) {
    ::kill(-pid, signal_number);
  }
}

int wait_process(pid_t pid, int timeout_ms = 5000) {
  if (pid <= 0) {
    return 0;
  }
  const auto start = std::chrono::steady_clock::now();
  int status = 0;
  while (true) {
    if (::waitpid(pid, &status, WNOHANG) == pid) {
      return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start).count();
    if (elapsed_ms > timeout_ms) {
      stop_process_group(pid, SIGTERM);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (::waitpid(pid, &status, WNOHANG) == pid) {
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      }
      stop_process_group(pid, SIGKILL);
      ::waitpid(pid, &status, 0);
      return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int run_shell_command_sync(const std::string & command, int timeout_ms = 60000) {
  const pid_t pid = spawn_shell_command(command, -1);
  return wait_process(pid, timeout_ms);
}

bool remove_path_recursive(const std::string & path) {
  if (!path_exists(path)) {
    return true;
  }
  // Forking one `rm -rf` process per rotated bag imposes a minimum 100 ms
  // wait in wait_process(). A large recovery backlog can therefore block the
  // recorder (and robot shutdown) for tens of minutes. C++17 remove_all keeps
  // the same no-symlink-following behavior without spawning child processes.
  std::error_code error;
  std::filesystem::remove_all(path, error);
  return !error;
}

std::string command_output(const std::string & command) {
  FILE * pipe = ::popen(command.c_str(), "r");
  if (!pipe) {
    return {};
  }
  std::ostringstream os;
  char buffer[512];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    os << buffer;
  }
  ::pclose(pipe);
  return trim(os.str());
}

std::string repair_script_path() {
  const std::string prefix = command_output("ros2 pkg prefix log_bag 2>/dev/null");
  if (!prefix.empty()) {
    const std::string script = join_path(prefix, "share/log_bag/scripts/repair_rosbag2.py");
    if (path_exists(script)) {
      return script;
    }
  }
  return {};
}

bool repair_bag_directory(const std::string & bag_path) {
  if (path_exists(join_path(bag_path, "metadata.yaml"))) {
    return true;
  }
  const std::string script = repair_script_path();
  if (!script.empty()) {
    const int repair_rc = run_shell_command_sync(
      "python3 " + shell_quote(script) + " " + shell_quote(bag_path), 300000);
    if (repair_rc == 0 && path_exists(join_path(bag_path, "metadata.yaml"))) {
      return true;
    }
  }
  const int reindex_rc = run_shell_command_sync("ros2 bag reindex " + shell_quote(bag_path), 120000);
  return reindex_rc == 0 && path_exists(join_path(bag_path, "metadata.yaml"));
}

std::string default_root() {
  const char * env_root = std::getenv("OPEN_DELIVERY_LOG_BAG_ROOT");
  if (env_root && *env_root) {
    return env_root;
  }
  const char * delivery_root = std::getenv("OPEN_DELIVERY_ROOT");
  if (delivery_root && *delivery_root) {
    return join_path(delivery_root, "log_bag");
  }
  char cwd[4096] {};
  if (::getcwd(cwd, sizeof(cwd)) != nullptr) {
    return join_path(cwd, "log_bag");
  }
  return "log_bag";
}

struct Config {
  std::string robot_name;
  std::string root = default_root();
  std::uintmax_t max_bag_bytes = kDefaultMaxBagBytes;
  std::uintmax_t max_robot_bytes = kDefaultMaxRobotBytes;
  std::uintmax_t prune_target_bytes = kDefaultPruneTargetBytes;
  double poll_seconds = 0.5;
};

void print_usage() {
  std::cerr
    << "Usage: robot_log_recorder --robot-name <name> [--root <log_bag>] "
       "[--max-bag-bytes <bytes>] [--max-robot-bytes <bytes>] "
       "[--prune-target-bytes <bytes>] [--poll-sec <seconds>]\n";
}

Config parse_args(int argc, char ** argv) {
  Config cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "--robot-name" || arg == "-r") && i + 1 < argc) {
      cfg.robot_name = argv[++i];
    } else if (arg == "--root" && i + 1 < argc) {
      cfg.root = argv[++i];
    } else if (arg == "--max-bag-bytes" && i + 1 < argc) {
      cfg.max_bag_bytes = static_cast<std::uintmax_t>(std::stoull(argv[++i]));
    } else if (arg == "--max-robot-bytes" && i + 1 < argc) {
      cfg.max_robot_bytes = static_cast<std::uintmax_t>(std::stoull(argv[++i]));
    } else if (arg == "--prune-target-bytes" && i + 1 < argc) {
      cfg.prune_target_bytes = static_cast<std::uintmax_t>(std::stoull(argv[++i]));
    } else if (arg == "--poll-sec" && i + 1 < argc) {
      cfg.poll_seconds = std::max(0.5, std::stod(argv[++i]));
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      std::exit(0);
    } else if (arg.rfind("--ros-args", 0) == 0) {
      break;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  cfg.robot_name = trim(cfg.robot_name);
  if (cfg.robot_name.empty()) {
    throw std::runtime_error("--robot-name is required");
  }
  if (cfg.max_bag_bytes == 0 || cfg.max_robot_bytes == 0 ||
    cfg.prune_target_bytes >= cfg.max_robot_bytes)
  {
    throw std::runtime_error(
      "bag sizes must be positive and prune target must be below robot limit");
  }
  return cfg;
}

class MatchIndex {
public:
  explicit MatchIndex(std::string path)
  : path_(std::move(path)) {
    load_previous_bags();
  }

  void add_bag(
    const std::string & robot_name,
    const std::string & bag_path,
    const std::string & text_log_path,
    const std::string & started_at,
    const std::string & ended_at,
    std::uintmax_t bytes,
    const std::vector<std::string> & topics,
    const std::vector<std::string> & tags,
    const std::string & reason) {
    load_previous_bags();
    new_bags_.clear();
    std::ostringstream os;
    os << "    \"" << json_escape(bag_path) << "\": {\n"
       << "      \"txt\": [\"" << json_escape(text_log_path) << "\"],\n"
       << "      \"tags\": [";
    for (std::size_t i = 0; i < tags.size(); ++i) {
      if (i > 0) {
        os << ", ";
      }
      os << "\"" << json_escape(tags[i]) << "\"";
    }
    os << "],\n"
       << "      \"robot_name\": \"" << json_escape(robot_name) << "\",\n"
       << "      \"started_at\": \"" << json_escape(started_at) << "\",\n"
       << "      \"ended_at\": \"" << json_escape(ended_at) << "\",\n"
       << "      \"bytes\": " << bytes << ",\n"
       << "      \"reason\": \"" << json_escape(reason) << "\",\n"
       << "      \"topics\": [";
    for (std::size_t i = 0; i < topics.size(); ++i) {
      if (i > 0) {
        os << ", ";
      }
      os << "\"" << json_escape(topics[i]) << "\"";
    }
    os << "]\n"
       << "    }";
    new_bags_.push_back(os.str());
    write();
  }

  std::vector<log_bag::MatchBagEntry> bag_entries() const {
    std::vector<log_bag::MatchBagEntry> entries =
      log_bag::parse_match_bag_members(previous_bags_raw_);
    for (const auto & raw : new_bags_) {
      const auto parsed = log_bag::parse_match_bag_members(raw);
      entries.insert(entries.end(), parsed.begin(), parsed.end());
    }

    std::vector<log_bag::MatchBagEntry> merged;
    for (const auto & entry : entries) {
      if (entry.path.empty()) {
        continue;
      }
      const auto existing = std::find_if(
        merged.begin(), merged.end(),
        [&entry](const log_bag::MatchBagEntry & candidate) {
          return candidate.path == entry.path;
        });
      if (existing == merged.end()) {
        merged.push_back(entry);
      } else if (entry.tagged || !existing->tagged) {
        // Prefer a tagged duplicate so an older valid tag can never be erased
        // by a later malformed/untagged duplicate index member.
        *existing = entry;
      }
    }
    return merged;
  }

  void remove_bags(const std::set<std::string> & paths) {
    if (paths.empty()) {
      return;
    }
    load_previous_bags();
    new_bags_.clear();
    std::vector<std::string> kept;
    const auto retain_members = [&paths, &kept](const std::string & raw_members) {
        for (const auto & entry : log_bag::parse_match_bag_members(raw_members)) {
          if (entry.path.empty() || paths.count(entry.path) == 0) {
            kept.push_back(entry.raw);
          }
        }
      };
    retain_members(previous_bags_raw_);
    for (const auto & raw : new_bags_) {
      retain_members(raw);
    }

    std::ostringstream os;
    for (std::size_t i = 0; i < kept.size(); ++i) {
      if (i > 0) {
        os << ",\n";
      }
      os << kept[i];
    }
    previous_bags_raw_ = os.str();
    new_bags_.clear();
    write();
  }

private:
  void load_previous_bags() {
    previous_bags_raw_.clear();
    std::ifstream in(path_);
    if (!in) {
      return;
    }
    std::ostringstream buf;
    buf << in.rdbuf();
    const std::string text = buf.str();
    const auto key = text.find("\"bags\"");
    if (key == std::string::npos) {
      return;
    }
    const auto open = text.find('{', key);
    if (open == std::string::npos) {
      return;
    }
    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (std::size_t i = open; i < text.size(); ++i) {
      const char ch = text[i];
      if (escaped) {
        escaped = false;
        continue;
      }
      if (ch == '\\') {
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
        ++depth;
      } else if (ch == '}') {
        --depth;
        if (depth == 0) {
          previous_bags_raw_ = trim(text.substr(open + 1, i - open - 1));
          return;
        }
      }
    }
  }

  void write() {
    ensure_dir(dirname_of(path_));
    const std::string tmp = path_ + ".tmp";
    std::ofstream out(tmp, std::ios::trunc);
    out << "{\n"
        << "  \"version\": 2,\n"
        << "  \"updated_at\": \"" << now_iso8601() << "\",\n"
        << "  \"bags\": {\n";
    bool wrote_any = false;
    if (!previous_bags_raw_.empty()) {
      out << previous_bags_raw_;
      wrote_any = true;
    }
    for (const auto & rec : new_bags_) {
      if (wrote_any) {
        out << ",\n";
      }
      out << rec;
      wrote_any = true;
    }
    out << "\n  }\n}\n";
    out.close();
    if (::rename(tmp.c_str(), path_.c_str()) != 0) {
      throw std::runtime_error("failed to update match.json: " + std::string(std::strerror(errno)));
    }
  }

  std::string path_;
  std::string previous_bags_raw_;
  std::vector<std::string> new_bags_;
};

class RobotLogRecorder {
public:
  explicit RobotLogRecorder(Config cfg)
  : cfg_(std::move(cfg)),
    robot_dir_(join_path(cfg_.root, cfg_.robot_name)),
    backup_dir_(join_path(robot_dir_, "backup")),
    backup_logs_dir_(join_path(backup_dir_, "logs")),
    backup_bags_dir_(join_path(backup_dir_, "bags")),
    match_(join_path(backup_dir_, "match.json")) {
    ensure_dir(robot_dir_);
    ensure_dir(backup_logs_dir_);
    ensure_dir(backup_bags_dir_);
    recover_leftover_artifacts();
    prune_untagged_backups();
    prune_storage_limit();
    open_text_log();
    ensure_ros();
  }

  ~RobotLogRecorder() {
    if (log_fd_ >= 0) {
      ::close(log_fd_);
    }
  }

  int run() {
    write_line(
      "robot_log_recorder starting for robot=" + cfg_.robot_name +
      " max_bag_bytes=" + std::to_string(cfg_.max_bag_bytes) +
      " max_robot_bytes=" + std::to_string(cfg_.max_robot_bytes) +
      " prune_target_bytes=" + std::to_string(cfg_.prune_target_bytes));
    while (!g_stop_requested.load()) {
      rclcpp::spin_some(node_);
      const auto heartbeat_now = std::chrono::steady_clock::now();
      if (has_latest_robot_status_ &&
        heartbeat_now - last_robot_status_received_steady_ >= kRobotStatusStall)
      {
        write_line("robot_status heartbeat stalled; restarting recorder process");
        stop_current_bag("heartbeat_stalled");
        archive_text_log();
        return 3;
      }
      write_bag_tags_marker(current_bag_path_, current_tags_);
      rotate_bag_if_needed();
      if (bag_pid_ <= 0) {
        start_next_bag_if_needed();
      }
      // Start the next recorder before any potentially slow deletion. The
      // rosbag child keeps writing while old backups and index rows are pruned.
      if (bag_pid_ > 0 && current_bag_health_verified_ && prune_pending_) {
        prune_untagged_backups();
        prune_storage_limit();
        prune_pending_ = false;
      }
      const auto now = std::chrono::steady_clock::now();
      if (bag_pid_ > 0 && current_bag_health_verified_ && now >= next_storage_check_) {
        prune_storage_limit();
        next_storage_check_ = now + kStorageCheckInterval;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(cfg_.poll_seconds * 1000.0)));
    }
    stop_current_bag("shutdown");
    archive_text_log();
    return 0;
  }

private:
  void ensure_ros() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
    node_ = std::make_shared<rclcpp::Node>(
      "robot_log_recorder", std::string("/") + cfg_.robot_name);

    rosout_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout",
      rclcpp::SystemDefaultsQoS(),
      [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
        write_rosout(*msg);
      });

    const auto task_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    task_status_sub_ = node_->create_subscription<custom_msgs_srvs::msg::TaskStatus>(
      "task_status",
      task_qos,
      [this](const custom_msgs_srvs::msg::TaskStatus::SharedPtr msg) {
        on_task_status(*msg);
      });

    robot_status_sub_ = node_->create_subscription<custom_msgs_srvs::msg::RobotStatus>(
      "robot_status",
      rclcpp::SystemDefaultsQoS(),
      [this](const custom_msgs_srvs::msg::RobotStatus::SharedPtr msg) {
        on_robot_status(*msg);
      });
  }

  static bool terminal_task_status(const std::string & status) {
    return status == custom_msgs_srvs::msg::TaskStatus::STATUS_FINISHED ||
           status == custom_msgs_srvs::msg::TaskStatus::STATUS_FAILED ||
           status == custom_msgs_srvs::msg::TaskStatus::STATUS_TERMINATED;
  }

  static bool append_unique(std::vector<std::string> & values, const std::string & value) {
    if (std::find(values.begin(), values.end(), value) != values.end()) {
      return false;
    }
    values.push_back(value);
    return true;
  }


  void on_robot_status(const custom_msgs_srvs::msg::RobotStatus & msg) {
    latest_robot_status_ = record_robot_status(msg);
    last_robot_status_received_steady_ = std::chrono::steady_clock::now();
    has_latest_robot_status_ = true;
    if (!current_bag_path_.empty()) {
      current_status_samples_.push_back(latest_robot_status_);
      if (current_status_samples_.size() > 10000) {
        current_status_samples_.erase(current_status_samples_.begin());
      }
    }
  }
  void on_task_status(const custom_msgs_srvs::msg::TaskStatus & msg) {
    const std::string task_id = trim(msg.task_id);
    if (task_id.empty()) {
      return;
    }

    if (terminal_task_status(msg.task_status)) {
      append_unique(current_tags_, task_id);
      active_task_ids_.erase(
        std::remove(active_task_ids_.begin(), active_task_ids_.end(), task_id),
        active_task_ids_.end());
      write_line("task terminal tag id=" + task_id + " status=" + msg.task_status);
      return;
    }

    const bool inserted = append_unique(active_task_ids_, task_id);
    append_unique(current_tags_, task_id);
    if (inserted) {
      write_line("task active tag id=" + task_id + " status=" + msg.task_status);
    }
  }

  void log_recovery(const std::string & line) const {
    std::cerr << "[robot_log_recorder recovery] " << line << "\n";
  }

  void recover_leftover_artifacts() {
    const auto entries = list_dir_basenames(robot_dir_);
    std::vector<std::string> leftover_logs;
    std::vector<std::string> leftover_bags;
    for (const auto & name : entries) {
      if (has_suffix(name, "_terminal_log.txt")) {
        leftover_logs.push_back(name);
      } else if (has_suffix(name, "_terminal_bag") && is_directory(join_path(robot_dir_, name))) {
        leftover_bags.push_back(name);
      }
    }
    if (leftover_logs.empty() && leftover_bags.empty()) {
      return;
    }
    log_recovery(
      "found leftover artifacts logs=" + std::to_string(leftover_logs.size()) +
      " bags=" + std::to_string(leftover_bags.size()));

    std::sort(leftover_logs.begin(), leftover_logs.end());
    for (const auto & name : leftover_logs) {
      const std::string from = join_path(robot_dir_, name);
      if (is_symlink(from)) {
        if (remove_path(from)) {
          log_recovery("removed leftover text log symlink: " + from);
        } else {
          log_recovery("failed to remove leftover text log symlink: " + from);
        }
        continue;
      }
      const std::string to = join_path(backup_logs_dir_, name);
      if (path_exists(to)) {
        log_recovery("removing stale active text log copy: " + from);
        remove_path(from);
        continue;
      }
      if (move_path(from, to)) {
        log_recovery("archived leftover text log: " + to);
      } else {
        log_recovery("failed to archive leftover text log: " + from);
      }
    }

    std::sort(leftover_bags.begin(), leftover_bags.end());
    for (const auto & name : leftover_bags) {
      const std::string bag_path = join_path(robot_dir_, name);
      const std::string archived = join_path(backup_bags_dir_, name);
      const std::string prefix = file_prefix_before_terminal(name);
      const std::string text_log_name = prefix + "_terminal_log.txt";
      const std::string text_log_path = join_path(backup_logs_dir_, text_log_name);

      if (path_exists(archived)) {
        log_recovery("backup bag already exists, removing stale active copy: " + bag_path);
        remove_path_recursive(bag_path);
        continue;
      }

      const bool repaired = repair_bag_directory(bag_path);
      if (!repaired) {
        log_recovery("bag reindex failed, archiving anyway: " + bag_path);
      } else {
        log_recovery("repaired bag: " + bag_path);
      }

      if (!move_path(bag_path, archived)) {
        log_recovery("failed to archive leftover bag: " + bag_path);
        continue;
      }

      const auto bytes = directory_size(archived);
      const std::string started_at = iso8601_from_compact_timestamp(prefix);
      const std::string ended_at = file_mtime_iso8601(archived);
      const std::string reason = repaired ? "startup_recovery" : "startup_recovery_unrepaired";
      const auto recovered_tags = read_bag_tags_marker(archived);
      match_.add_bag(
        cfg_.robot_name,
        archived,
        text_log_path,
        started_at,
        ended_at,
        bytes,
        {},
        recovered_tags,
        reason);
      log_recovery("indexed leftover bag: " + archived + " reason=" + reason);
    }
  }

  void prune_untagged_backups() {
    struct Candidate {
      bool tagged{false};
      std::set<std::string> index_paths;
    };

    const auto indexed = match_.bag_entries();
    std::map<std::string, Candidate> candidates;
    std::set<std::string> remove_from_index;
    for (const auto & entry : indexed) {
      // Never derive a deletion target from an arbitrary index path. Only
      // accept records that identify a backup/bags member, then reconstruct the
      // actual target under this recorder's own backup_bags_dir_.
      if (entry.path.find("/backup/bags/") == std::string::npos) {
        continue;
      }
      const std::string name = basename_of(entry.path);
      if (!same_existing_path(dirname_of(entry.path), backup_bags_dir_)) {
        continue;
      }
      if (name.empty() || name.find('/') != std::string::npos ||
        !has_suffix(name, "_terminal_bag"))
      {
        continue;
      }
      const std::string target = join_path(backup_bags_dir_, name);
      if (!is_directory(target)) {
        remove_from_index.insert(entry.path);
        continue;
      }
      auto & candidate = candidates[name];
      candidate.tagged = candidate.tagged || entry.tagged;
      candidate.index_paths.insert(entry.path);
    }

    std::vector<std::string> names;
    std::vector<bool> tagged;
    names.reserve(candidates.size());
    tagged.reserve(candidates.size());
    for (const auto & item : candidates) {
      names.push_back(item.first);
      tagged.push_back(item.second.tagged);
    }
    const auto keep = log_bag::retention_keep_mask(tagged);
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (g_stop_requested.load()) {
        log_recovery("retention interrupted by shutdown request");
        break;
      }
      if (keep[i] || tagged[i]) {
        continue;
      }
      const std::string & name = names[i];
      const std::string target = join_path(backup_bags_dir_, name);
      // This is the final guard immediately before deletion.
      if (basename_of(target) != name || !has_suffix(name, "_terminal_bag") ||
        !same_existing_path(dirname_of(target), backup_bags_dir_) ||
        !is_directory(target))
      {
        continue;
      }
      if (remove_path_recursive(target)) {
        remove_from_index.insert(
          candidates[name].index_paths.begin(), candidates[name].index_paths.end());
        if (log_fd_ >= 0) {
          write_line("retention removed untagged backup bag path=" + target);
        } else {
          log_recovery("retention removed untagged backup bag: " + target);
        }
      } else if (log_fd_ >= 0) {
        write_line("retention failed to remove untagged backup bag path=" + target);
      } else {
        log_recovery("retention failed to remove untagged backup bag: " + target);
      }
    }
    match_.remove_bags(remove_from_index);
  }

  void prune_storage_limit() {
    std::vector<std::string> names;
    std::vector<std::uintmax_t> sizes;
    for (const auto & name : list_dir_basenames(backup_bags_dir_)) {
      const std::string target = join_path(backup_bags_dir_, name);
      if (name.find('/') == std::string::npos && has_suffix(name, "_terminal_bag") &&
        is_directory(target))
      {
        names.push_back(name);
      }
    }
    std::sort(names.begin(), names.end());
    std::uintmax_t archived_bytes = 0;
    for (const auto & name : names) {
      const auto bytes = directory_size(join_path(backup_bags_dir_, name));
      sizes.push_back(bytes);
      archived_bytes += bytes;
    }
    const auto active_bytes = current_bag_path_.empty()
      ? 0U : directory_size(current_bag_path_);
    if (archived_bytes <= cfg_.max_robot_bytes &&
      active_bytes <= cfg_.max_robot_bytes - archived_bytes)
    {
      return;
    }

    if (log_fd_ >= 0) {
      write_line(
        "storage cap triggered archived_bytes=" + std::to_string(archived_bytes) +
        " active_bytes=" + std::to_string(active_bytes) +
        " max_robot_bytes=" + std::to_string(cfg_.max_robot_bytes) +
        " prune_target_bytes=" + std::to_string(cfg_.prune_target_bytes));
    } else {
      log_recovery(
        "storage cap triggered archived_bytes=" + std::to_string(archived_bytes));
    }

    const auto indexed = match_.bag_entries();
    std::map<std::string, std::set<std::string>> index_paths_by_name;
    for (const auto & entry : indexed) {
      if (entry.path.find("/backup/bags/") != std::string::npos) {
        index_paths_by_name[basename_of(entry.path)].insert(entry.path);
      }
    }
    const auto archived_target = active_bytes < cfg_.prune_target_bytes
      ? cfg_.prune_target_bytes - active_bytes : 0U;
    const auto keep = log_bag::storage_keep_mask(sizes, archived_target);
    std::set<std::string> remove_from_index;
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (keep[i]) {
        continue;
      }
      const std::string & name = names[i];
      const std::string target = join_path(backup_bags_dir_, name);
      if (basename_of(target) != name || !has_suffix(name, "_terminal_bag") ||
        !same_existing_path(dirname_of(target), backup_bags_dir_) || !is_directory(target))
      {
        continue;
      }
      if (remove_path_recursive(target)) {
        const auto found = index_paths_by_name.find(name);
        if (found != index_paths_by_name.end()) {
          remove_from_index.insert(found->second.begin(), found->second.end());
        }
        if (log_fd_ >= 0) {
          write_line(
            "storage cap removed oldest bag path=" + target +
            " bytes=" + std::to_string(sizes[i]));
        } else {
          log_recovery("storage cap removed oldest bag: " + target);
        }
      }
    }
    match_.remove_bags(remove_from_index);
  }

  void open_text_log() {
    const std::string ts = timestamp_for_filename();
    const std::string name = ts + "_terminal_log.txt";
    text_log_real_path_ = join_path(backup_logs_dir_, name);
    text_log_link_path_ = join_path(robot_dir_, name);
    log_fd_ = ::open(text_log_real_path_.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
    if (log_fd_ < 0) {
      throw std::runtime_error("open log failed: " + std::string(std::strerror(errno)));
    }
    const std::string rel_target = join_path("backup/logs", name);
    if (!create_symlink(rel_target, text_log_link_path_)) {
      throw std::runtime_error(
        "create text log symlink failed: " + text_log_link_path_ + ": " + std::strerror(errno));
    }
  }

  void write_line(const std::string & line) {
    const std::string out = "[" + now_iso8601() + "] " + line + "\n";
    if (log_fd_ >= 0) {
      ::write(log_fd_, out.data(), out.size());
    }
  }

  void write_rosout(const rcl_interfaces::msg::Log & msg) {
    std::ostringstream os;
    os << "rosout"
       << " level=" << static_cast<int>(msg.level)
       << " name=" << msg.name
       << " file=" << msg.file
       << " line=" << msg.line
       << " msg=" << msg.msg;
    write_line(os.str());
  }

  std::vector<std::string> robot_topics() {
    return log_bag::recording_topics(cfg_.robot_name);
  }

  std::string next_bag_path() const {
    const std::string timestamp = timestamp_for_filename();
    for (unsigned int sequence = 0; sequence < 1000; ++sequence) {
      std::ostringstream name;
      name << timestamp;
      if (sequence > 0) {
        name << "_" << std::setw(3) << std::setfill('0') << sequence;
      }
      name << "_terminal_bag";
      const std::string active = join_path(robot_dir_, name.str());
      const std::string backup = join_path(backup_bags_dir_, name.str());
      if (!path_exists(active) && !path_exists(backup)) {
        return active;
      }
    }
    throw std::runtime_error("unable to allocate unique bag path");
  }

  void start_next_bag_if_needed() {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_bag_start_not_before_) {
      return;
    }
    const auto topics = robot_topics();
    const std::string critical_status_topic = "/" + cfg_.robot_name + "/robot_status";
    if (!has_latest_robot_status_) {
      write_line("critical topic has no heartbeat yet; bag recorder waits topic=" +
        critical_status_topic);
      return;
    }
    const char * configured_root = std::getenv("OPEN_DELIVERY_ROOT");
    const std::string workspace_root = configured_root && *configured_root
      ? configured_root : dirname_of(cfg_.root);
    const std::string custom_msgs_prefix = find_custom_msgs_prefix(workspace_root);
    if (custom_msgs_prefix.empty()) {
      write_line(
        "custom_msgs_srvs ament prefix unavailable; bag recorder waits instead of "
        "creating a rosout-only bag");
      return;
    }
    current_bag_topics_ = topics;
    current_tags_ = active_task_ids_;
    current_bag_started_at_ = now_iso8601();
    current_bag_path_ = next_bag_path();
    current_status_samples_.clear();
    if (has_latest_robot_status_) {
      latest_robot_status_.timestamp_ns = system_now_ns();
      current_status_samples_.push_back(latest_robot_status_);
    }
    current_bag_started_steady_ = now;
    next_bag_health_check_ = now + kCriticalTopicGrace;
    last_critical_progress_ = now;
    last_critical_message_count_ = 0;
    current_bag_health_verified_ = false;
    std::ostringstream cmd;
    cmd << "export AMENT_PREFIX_PATH=" << shell_quote(custom_msgs_prefix)
        << "${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}; "
        << "export LD_LIBRARY_PATH=" << shell_quote(join_path(custom_msgs_prefix, "lib"))
        << "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}; "
        << "exec ros2 bag record -o " << shell_quote(current_bag_path_);
    for (const auto & topic : topics) {
      cmd << " " << shell_quote(topic);
    }
    write_line("starting bag recorder: " + cmd.str());
    bag_pid_ = spawn_shell_command(cmd.str(), log_fd_);
  }

  void restart_unhealthy_bag(const std::string & reason, std::uintmax_t status_messages)
  {
    consecutive_unhealthy_bags_ = std::min(5U, consecutive_unhealthy_bags_ + 1U);
    const unsigned int exponent = std::min(
      4U, consecutive_unhealthy_bags_ - 1U);
    const auto backoff = std::min(
      kMaxRecorderRestartBackoff, std::chrono::seconds(5U * (1U << exponent)));
    write_line(
      "bag unhealthy: " + reason +
      " robot_status_messages=" + std::to_string(status_messages) +
      " retry_in_seconds=" + std::to_string(backoff.count()));
    stop_process_group(bag_pid_, SIGINT);
    wait_process(bag_pid_);
    bag_pid_ = -1;
    const auto failed_bytes = directory_size(current_bag_path_);
    if (current_tags_.empty()) {
      discard_current_bag(reason, failed_bytes);
    } else {
      archive_current_bag(reason);
    }
    next_bag_start_not_before_ = std::chrono::steady_clock::now() + backoff;
  }

  void rotate_bag_if_needed() {
    if (bag_pid_ <= 0) {
      return;
    }
    int status = 0;
    if (process_exited(bag_pid_, &status)) {
      write_line("bag recorder exited with status=" + std::to_string(status));
      bag_pid_ = -1;
      archive_current_bag("recorder_exit");
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto recording_age =
      now - current_bag_started_steady_;
    if (now >= next_bag_health_check_)
    {
      const std::string critical_status_topic = "/" + cfg_.robot_name + "/robot_status";
      std::uintmax_t status_messages = 0;
      const bool count_ready = sqlite_topic_message_count(
        current_bag_path_, critical_status_topic, &status_messages);
      if (count_ready) {
        if (status_messages > last_critical_message_count_) {
          last_critical_message_count_ = status_messages;
          last_critical_progress_ = now;
          current_bag_health_verified_ = true;
          consecutive_unhealthy_bags_ = 0;
        }
        const bool heartbeat_fresh = has_latest_robot_status_ &&
          now - last_robot_status_received_steady_ < kCriticalTopicGrace;
        const bool missing_initial_status = !current_bag_health_verified_ &&
          recording_age >= kCriticalTopicGrace;
        const bool status_stalled = current_bag_health_verified_ &&
          now - last_critical_progress_ >= kCriticalTopicStall;
        if (heartbeat_fresh && (missing_initial_status || status_stalled)) {
          const std::string reason = missing_initial_status
            ? "missing_robot_status" : "stalled_robot_status";
          restart_unhealthy_bag(reason, status_messages);
          return;
        }
      }
      next_bag_health_check_ = now + kCriticalTopicCheckInterval;
    }
    const auto logical_bytes = sqlite_logical_bag_size(current_bag_path_);
    const auto bytes = logical_bytes ? logical_bytes : directory_size(current_bag_path_);
    if (bytes >= cfg_.max_bag_bytes) {
      write_line("bag reached max size; rotating logical_bytes=" + std::to_string(bytes));
      stop_current_bag("size_limit");
    }
  }

  void stop_current_bag(const std::string & reason) {
    if (bag_pid_ > 0) {
      stop_process_group(bag_pid_, SIGINT);
      wait_process(bag_pid_);
      bag_pid_ = -1;
    }
    archive_current_bag(reason);
  }

  void discard_current_bag(const std::string & reason, std::uintmax_t bytes) {
    write_line(
      "discarding bag reason=" + reason + " bytes=" + std::to_string(bytes) + " path=" +
      current_bag_path_);
    remove_path_recursive(current_bag_path_);
    current_bag_path_.clear();
    current_bag_topics_.clear();
    current_bag_started_at_.clear();
    current_tags_.clear();
    current_status_samples_.clear();
  }

  void archive_current_bag(const std::string & reason) {
    if (current_bag_path_.empty() || !path_exists(current_bag_path_)) {
      current_bag_path_.clear();
      current_bag_topics_.clear();
      current_bag_started_at_.clear();
      current_tags_.clear();
      current_status_samples_.clear();
      return;
    }
    write_robot_status_sidecar(current_bag_path_, current_status_samples_);
    const auto bytes = directory_size(current_bag_path_);
    if (reason == "recorder_exit" && bytes < kMinArchiveBagBytes) {
      discard_current_bag(reason, bytes);
      return;
    }
    const std::string archived = join_path(backup_bags_dir_, basename_of(current_bag_path_));
    if (path_exists(archived)) {
      write_line("backup bag already exists; discarding active copy path=" + current_bag_path_);
      discard_current_bag("duplicate_active", bytes);
      return;
    }
    write_bag_tags_marker(current_bag_path_, current_tags_);
    repair_bag_directory(current_bag_path_);
    if (!move_path(current_bag_path_, archived)) {
      throw std::runtime_error(
        "failed to move bag into backup directory: " + current_bag_path_);
    }
    match_.add_bag(
      cfg_.robot_name,
      archived,
      intended_archived_text_log_path(),
      current_bag_started_at_,
      now_iso8601(),
      bytes,
      current_bag_topics_,
      current_tags_,
      reason);
    current_bag_path_.clear();
    current_bag_topics_.clear();
    current_bag_started_at_.clear();
    current_tags_.clear();

    current_status_samples_.clear();
    prune_pending_ = true;
  }

  void archive_text_log() {
    if (text_log_real_path_.empty() || !archived_text_log_path_.empty()) {
      return;
    }
    ::fsync(log_fd_);
    ::close(log_fd_);
    log_fd_ = -1;
    if (!text_log_link_path_.empty() && is_symlink(text_log_link_path_)) {
      remove_path(text_log_link_path_);
    }
    archived_text_log_path_ = text_log_real_path_;
  }

  std::string intended_archived_text_log_path() const {
    if (!archived_text_log_path_.empty()) {
      return archived_text_log_path_;
    }
    if (!text_log_real_path_.empty()) {
      return text_log_real_path_;
    }
    return {};
  }

  Config cfg_;
  std::string robot_dir_;
  std::string backup_dir_;
  std::string backup_logs_dir_;
  std::string backup_bags_dir_;
  MatchIndex match_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  int log_fd_{-1};
  rclcpp::Subscription<custom_msgs_srvs::msg::TaskStatus>::SharedPtr task_status_sub_;
  rclcpp::Subscription<custom_msgs_srvs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  RecordedRobotStatus latest_robot_status_;
  bool has_latest_robot_status_{false};
  std::chrono::steady_clock::time_point last_robot_status_received_steady_{};
  std::vector<RecordedRobotStatus> current_status_samples_;
  std::vector<std::string> active_task_ids_;
  std::string text_log_real_path_;
  std::string text_log_link_path_;
  std::string archived_text_log_path_;
  pid_t bag_pid_{-1};
  std::string current_bag_path_;
  std::string current_bag_started_at_;
  std::vector<std::string> current_bag_topics_;
  std::vector<std::string> current_tags_;
  std::chrono::steady_clock::time_point current_bag_started_steady_{};
  std::chrono::steady_clock::time_point next_bag_health_check_{};
  std::chrono::steady_clock::time_point last_critical_progress_{};
  std::chrono::steady_clock::time_point next_bag_start_not_before_{};
  std::uintmax_t last_critical_message_count_{0};
  unsigned int consecutive_unhealthy_bags_{0};
  std::chrono::steady_clock::time_point next_storage_check_{};
  bool current_bag_health_verified_{false};
  bool prune_pending_{false};
};

}  // namespace

int main(int argc, char ** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  try {
    Config cfg = parse_args(argc, argv);
    RobotLogRecorder recorder(std::move(cfg));
    const int rc = recorder.run();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return rc;
  } catch (const std::exception & e) {
    std::cerr << "robot_log_recorder: " << e.what() << "\n";
    print_usage();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 2;
  }
}
