#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdio>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr std::uintmax_t kDefaultMaxBagBytes = 100U * 1024U * 1024U;
constexpr std::uintmax_t kMinArchiveBagBytes = 64U * 1024U;

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

std::string now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm {};
  gmtime_r(&t, &tm);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return os.str();
}

std::string timestamp_for_filename() {
  const auto now = std::chrono::system_clock::now();
  const auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm {};
  gmtime_r(&t, &tm);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y%m%dT%H%M%S");
  return os.str();
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

std::string file_prefix_before_terminal(const std::string & name) {
  const std::string marker = "_terminal";
  const auto pos = name.find(marker);
  if (pos == std::string::npos) {
    return {};
  }
  return name.substr(0, pos);
}

std::string iso8601_from_compact_timestamp(const std::string & compact) {
  if (compact.size() != 15 || compact[8] != 'T') {
    return now_iso8601();
  }
  return compact.substr(0, 4) + "-" + compact.substr(4, 2) + "-" + compact.substr(6, 2) + "T" +
         compact.substr(9, 2) + ":" + compact.substr(11, 2) + ":" + compact.substr(13, 2) + "Z";
}

std::string file_mtime_iso8601(const std::string & path) {
  struct stat st {};
  if (::stat(path.c_str(), &st) != 0) {
    return now_iso8601();
  }
  std::tm tm {};
  gmtime_r(&st.st_mtime, &tm);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return os.str();
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
  const int rc = run_shell_command_sync("rm -rf " + shell_quote(path), 30000);
  return rc == 0;
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
  double poll_seconds = 2.0;
};

void print_usage() {
  std::cerr
    << "Usage: robot_log_recorder --robot-name <name> [--root <log_bag>] "
       "[--max-bag-bytes <bytes>] [--poll-sec <seconds>]\n";
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

private:
  void load_previous_bags() {
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
    open_text_log();
    ensure_ros();
  }

  ~RobotLogRecorder() {
    if (log_fd_ >= 0) {
      ::close(log_fd_);
    }
  }

  int run() {
    write_line("robot_log_recorder starting for robot=" + cfg_.robot_name);
    while (!g_stop_requested.load()) {
      rclcpp::spin_some(node_);
      rotate_bag_if_needed();
      if (bag_pid_ <= 0) {
        start_next_bag_if_needed();
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
      match_.add_bag(
        cfg_.robot_name,
        archived,
        text_log_path,
        started_at,
        ended_at,
        bytes,
        {},
        {},
        reason);
      log_recovery("indexed leftover bag: " + archived + " reason=" + reason);
    }
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
    std::vector<std::string> topics;
    try {
      const auto graph = node_->get_topic_names_and_types();
      const std::string ns_prefix = "/" + cfg_.robot_name + "/";
      for (const auto & item : graph) {
        const std::string & topic = item.first;
        if (topic == "/rosout" || topic == "/tf" || topic == "/tf_static" ||
          topic == "/clock" || topic == "/gazebo/model_states" ||
          topic.find(ns_prefix) == 0 || topic.find(ns_prefix) != std::string::npos)
        {
          topics.push_back(topic);
        }
      }
    } catch (const std::exception & e) {
      write_line(std::string("topic discovery failed: ") + e.what());
    }
    std::sort(topics.begin(), topics.end());
    topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
    return topics;
  }

  void start_next_bag_if_needed() {
    const auto topics = robot_topics();
    if (topics.empty()) {
      write_line("no robot-related topics discovered yet; bag recorder waits");
      return;
    }
    current_bag_topics_ = topics;
    current_bag_started_at_ = now_iso8601();
    current_bag_path_ = join_path(robot_dir_, timestamp_for_filename() + "_terminal_bag");
    std::ostringstream cmd;
    cmd << "ros2 bag record -o " << shell_quote(current_bag_path_);
    for (const auto & topic : topics) {
      cmd << " " << shell_quote(topic);
    }
    write_line("starting bag recorder: " + cmd.str());
    bag_pid_ = spawn_shell_command(cmd.str(), log_fd_);
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
    const auto bytes = directory_size(current_bag_path_);
    if (bytes >= cfg_.max_bag_bytes) {
      write_line("bag reached max size; rotating bytes=" + std::to_string(bytes));
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
  }

  void archive_current_bag(const std::string & reason) {
    if (current_bag_path_.empty() || !path_exists(current_bag_path_)) {
      current_bag_path_.clear();
      current_bag_topics_.clear();
      return;
    }
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
    repair_bag_directory(current_bag_path_);
    move_path(current_bag_path_, archived);
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
  std::string text_log_real_path_;
  std::string text_log_link_path_;
  std::string archived_text_log_path_;
  pid_t bag_pid_{-1};
  std::string current_bag_path_;
  std::string current_bag_started_at_;
  std::vector<std::string> current_bag_topics_;
  std::vector<std::string> current_tags_;
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
