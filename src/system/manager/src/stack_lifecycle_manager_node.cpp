#include "manager/stack_lifecycle_manager_node.hpp"

#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <future>
#include <thread>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace {

bool file_exists(const std::string & path) {
  struct stat st {};
  return !path.empty() && ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

std::string strip(const std::string & s) {
  const char * ws = " \t\n\r";
  const auto a = s.find_first_not_of(ws);
  if (a == std::string::npos) {
    return {};
  }
  const auto b = s.find_last_not_of(ws);
  return s.substr(a, b - a + 1);
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

std::string leaf_namespace(const std::string & ns) {
  std::string n = strip(ns);
  while (!n.empty() && n.front() == '/') {
    n.erase(0, 1);
  }
  if (n.empty()) {
    return {};
  }
  const auto pos = n.rfind('/');
  if (pos == std::string::npos) {
    return n;
  }
  return n.substr(pos + 1);
}

uint8_t lifecycle_transition_id(const std::string & transition) {
  const std::string t = lower(transition);
  if (t == "configure") {
    return lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  }
  if (t == "cleanup") {
    return lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
  }
  if (t == "activate") {
    return lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  }
  if (t == "deactivate") {
    return lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
  }
  if (t == "shutdown") {
    return lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
  }
  return 0;
}

std::string lifecycle_state_label(uint8_t id) {
  switch (id) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      return "unconfigured";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      return "inactive";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      return "active";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      return "finalized";
    default:
      return "unknown";
  }
}

template<typename FutureT>
bool wait_future(const FutureT & fut, std::chrono::nanoseconds timeout) {
  // Do not spin_some here: this node runs inside MultiThreadedExecutor; spinning the
  // same node interface throws "Node has already been added to an executor".
  const auto step = std::chrono::milliseconds(20);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (fut.wait_for(step) == std::future_status::ready) {
      return true;
    }
    std::this_thread::sleep_for(step);
  }
  return fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

}  // namespace

namespace manager {

StackLifecycleManagerNode::StackLifecycleManagerNode()
: rclcpp::Node("stack_lifecycle_manager"), slam_mode_("inactive") {
  // Launch may already declare use_sim_time via parameters=[{...}]; re-declare crashes on Foxy.
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", true);
  }
  declare_parameter<std::string>("mapper_params_file", "");
  declare_parameter<std::string>("localization_params_file", "");
  declare_parameter<std::string>("map_file", "");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("odom_frame", "");
  declare_parameter<std::string>("base_frame", "");
  declare_parameter<std::string>("scan_topic", "");
  declare_parameter<std::string>("mapping_map_topic", "");
  declare_parameter<std::string>("localization_map_topic", "");
  declare_parameter<std::string>("static_map_topic", "");
  declare_parameter<std::string>("slam_child_namespace", "");
  declare_parameter<std::string>("initial_slam_mode", "inactive");
  declare_parameter<std::vector<std::string>>(
    "tracked_lifecycle_nodes",
    std::vector<std::string>{"heartbeat", "lifecycle_manager_navigation", "map_server"});

  use_sim_time_ = get_parameter("use_sim_time").as_bool();
  mapper_params_file_ = get_parameter("mapper_params_file").as_string();
  localization_params_file_ = get_parameter("localization_params_file").as_string();
  map_file_ = get_parameter("map_file").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  mapping_map_topic_ = get_parameter("mapping_map_topic").as_string();
  localization_map_topic_ = get_parameter("localization_map_topic").as_string();
  static_map_topic_ = get_parameter("static_map_topic").as_string();
  slam_child_namespace_ = get_parameter("slam_child_namespace").as_string();
  tracked_lifecycle_nodes_ = get_parameter("tracked_lifecycle_nodes").as_string_array();

  const std::string rid = robot_id();
  if (odom_frame_.empty()) {
    odom_frame_ = rid + "/odom";
  }
  if (base_frame_.empty()) {
    base_frame_ = rid + "/base_footprint";
  }
  if (scan_topic_.empty()) {
    scan_topic_ = std::string("/") + rid + "/scan_2d";
  }
  if (mapping_map_topic_.empty()) {
    mapping_map_topic_ = std::string("/") + rid + "/mapping";
  }
  // SLAM localization must not own /<robot>/map — that is map_server's static occupancy grid.
  if (localization_map_topic_.empty()) {
    localization_map_topic_ = std::string("/") + rid + "/slam_map";
  }
  if (static_map_topic_.empty()) {
    static_map_topic_ = std::string("/") + rid + "/map";
  }
  if (slam_child_namespace_.empty()) {
    slam_child_namespace_ = std::string("/") + rid + "/slam_bringup";
  }

  client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  stack_pub_ = create_publisher<custom_msgs_srvs::msg::StackLifecycle>("stack_lifecycle", 10);
  transition_srv_ = create_service<custom_msgs_srvs::srv::SetStackLifecycleTransition>(
    "set_stack_lifecycle_transition",
    std::bind(
      &StackLifecycleManagerNode::handle_transition, this, std::placeholders::_1,
      std::placeholders::_2));
  publish_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&StackLifecycleManagerNode::publish_stack_lifecycle, this));

  // Load static floor map whenever map_file (occupancy yaml) is provided, including mapping mode
  // so /<robot>/map stays the saved map and is never overwritten by /mapping.
  if (!map_file_.empty()) {
    std::string map_err;
    if (!ensure_static_map_loaded(&map_err)) {
      RCLCPP_WARN(get_logger(), "static map load failed: %s", map_err.c_str());
    }
  }

  const std::string initial = lower(get_parameter("initial_slam_mode").as_string());
  if (!initial.empty() && initial != "inactive") {
    std::string err;
    if (!set_slam_mode(initial, &err)) {
      RCLCPP_WARN(get_logger(), "initial_slam_mode=%s failed: %s", initial.c_str(), err.c_str());
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "stack_lifecycle_manager ready (robot=%s slam_mode=%s static_map=%s slam_map=%s mapping=%s)",
    rid.c_str(), slam_mode_.c_str(), static_map_topic_.c_str(), localization_map_topic_.c_str(),
    mapping_map_topic_.c_str());
}

StackLifecycleManagerNode::~StackLifecycleManagerNode() {
  stop_slam_child();
  stop_map_server();
}

std::string StackLifecycleManagerNode::robot_id() const {
  const std::string leaf = leaf_namespace(get_namespace());
  return leaf.empty() ? std::string("robot") : leaf;
}

std::string StackLifecycleManagerNode::resolve_lifecycle_node_fqn(const std::string & node_name) const {
  std::string n = strip(node_name);
  if (n.empty()) {
    return {};
  }
  if (!n.empty() && n.front() == '/') {
    return n;
  }
  const std::string rid = robot_id();
  return std::string("/") + rid + "/" + n;
}

std::string StackLifecycleManagerNode::query_lifecycle_state(const std::string & node_fqn) {
  try {
    auto client = create_client<lifecycle_msgs::srv::GetState>(
      node_fqn + "/get_state", rmw_qos_profile_services_default, client_cb_group_);
    if (!client->wait_for_service(std::chrono::milliseconds(200))) {
      return "missing";
    }
    auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto fut = client->async_send_request(req);
    if (!wait_future(fut, std::chrono::milliseconds(500))) {
      return "timeout";
    }
    auto resp = fut.get();
    if (!resp) {
      return "error";
    }
    return lifecycle_state_label(resp->current_state.id);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "query_lifecycle_state(%s): %s", node_fqn.c_str(),
      e.what());
    return "error";
  }
}

bool StackLifecycleManagerNode::call_lifecycle_transition(
  const std::string & node_fqn,
  const std::string & transition,
  std::string * err) {
  const uint8_t tid = lifecycle_transition_id(transition);
  if (tid == 0) {
    if (err) {
      *err = "unknown lifecycle transition: " + transition;
    }
    return false;
  }
  auto client = create_client<lifecycle_msgs::srv::ChangeState>(
    node_fqn + "/change_state", rmw_qos_profile_services_default, client_cb_group_);
  if (!client->wait_for_service(std::chrono::seconds(3))) {
    if (err) {
      *err = "lifecycle service missing for " + node_fqn;
    }
    return false;
  }
  auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  req->transition.id = tid;
  auto fut = client->async_send_request(req);
  if (!wait_future(fut, std::chrono::seconds(8))) {
    if (err) {
      *err = "lifecycle transition timeout for " + node_fqn;
    }
    return false;
  }
  auto resp = fut.get();
  if (!resp || !resp->success) {
    if (err) {
      *err = "lifecycle transition rejected for " + node_fqn;
    }
    return false;
  }
  return true;
}

void StackLifecycleManagerNode::stop_map_server() {
  if (map_server_pid_ <= 0) {
    map_server_yaml_.clear();
    return;
  }
  RCLCPP_INFO(get_logger(), "Stopping map_server child pid=%d", map_server_pid_);
  kill(map_server_pid_, SIGINT);
  constexpr int max_wait_cycles = 50;
  for (int i = 0; i < max_wait_cycles; ++i) {
    int status = 0;
    const pid_t ret = waitpid(map_server_pid_, &status, WNOHANG);
    if (ret == map_server_pid_ || ret < 0) {
      map_server_pid_ = -1;
      map_server_yaml_.clear();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  kill(map_server_pid_, SIGKILL);
  waitpid(map_server_pid_, nullptr, 0);
  map_server_pid_ = -1;
  map_server_yaml_.clear();
}

std::string StackLifecycleManagerNode::resolve_posegraph_stem() const {
  // slam_toolbox appends ".posegraph" / ".data" to map_file_name. Occupancy yaml is NOT a posegraph.
  std::string stem = map_file_;
  const auto slash = stem.find_last_of('/');
  const std::string base = (slash == std::string::npos) ? stem : stem.substr(slash + 1);
  const auto dot = base.find_last_of('.');
  if (dot != std::string::npos) {
    const std::string ext = lower(base.substr(dot));
    if (ext == ".yaml" || ext == ".yml") {
      stem = stem.substr(0, stem.size() - (base.size() - dot));
    }
  }
  if (file_exists(stem + ".posegraph") && file_exists(stem + ".data")) {
    return stem;
  }
  return {};
}

bool StackLifecycleManagerNode::start_map_server(std::string * err) {
  if (map_file_.empty()) {
    if (err) {
      *err = "map_file is empty";
    }
    return false;
  }
  if (!file_exists(map_file_)) {
    if (err) {
      *err = "map yaml missing: " + map_file_;
    }
    return false;
  }

  std::string exec_path;
  try {
    exec_path = ament_index_cpp::get_package_prefix("nav2_map_server") + "/lib/nav2_map_server/map_server";
  } catch (const std::exception & e) {
    if (err) {
      *err = std::string("nav2_map_server path: ") + e.what();
    }
    return false;
  }
  if (access(exec_path.c_str(), X_OK) != 0) {
    if (err) {
      *err = "map_server executable missing: " + exec_path;
    }
    return false;
  }

  const std::string rid = robot_id();
  // topic_name is relative under PushRosNamespace-equivalent __ns so it becomes /<rid>/map.
  // Prefer absolute static_map_topic_ leaf "map" when under robot namespace.
  std::string topic_leaf = "map";
  {
    const std::string prefix = std::string("/") + rid + "/";
    if (static_map_topic_.rfind(prefix, 0) == 0) {
      topic_leaf = static_map_topic_.substr(prefix.size());
    } else if (!static_map_topic_.empty() && static_map_topic_.front() != '/') {
      topic_leaf = static_map_topic_;
    }
  }

  std::vector<std::string> args = {
    exec_path,
    "--ros-args",
    "-r", "__node:=map_server",
    "-r", "__ns:=" + std::string("/") + rid,
    "-p", "use_sim_time:=" + std::string(use_sim_time_ ? "true" : "false"),
    "-p", "yaml_filename:=" + map_file_,
    "-p", "topic_name:=" + topic_leaf,
    "-p", "frame_id:=" + map_frame_,
  };

  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (auto & item : args) {
    argv.push_back(const_cast<char *>(item.c_str()));
  }
  argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    if (err) {
      *err = "fork() failed for map_server";
    }
    return false;
  }
  if (pid == 0) {
    execv(argv[0], argv.data());
    _exit(127);
  }

  map_server_pid_ = pid;
  map_server_yaml_ = map_file_;
  RCLCPP_INFO(
    get_logger(), "Started map_server pid=%d yaml=%s topic=/%s/%s",
    map_server_pid_, map_file_.c_str(), rid.c_str(), topic_leaf.c_str());

  const std::string fqn = resolve_lifecycle_node_fqn("map_server");
  std::string local_err;
  // Wait until lifecycle services appear.
  bool ready = false;
  for (int i = 0; i < 50; ++i) {
    auto client = create_client<lifecycle_msgs::srv::ChangeState>(
      fqn + "/change_state", rmw_qos_profile_services_default, client_cb_group_);
    if (client->wait_for_service(std::chrono::milliseconds(100))) {
      ready = true;
      break;
    }
  }
  if (!ready) {
    if (err) {
      *err = "map_server lifecycle service not ready";
    }
    return false;
  }
  if (!call_lifecycle_transition(fqn, "configure", &local_err)) {
    if (err) {
      *err = "map_server configure failed: " + local_err;
    }
    return false;
  }
  if (!call_lifecycle_transition(fqn, "activate", &local_err)) {
    if (err) {
      *err = "map_server activate failed: " + local_err;
    }
    return false;
  }
  RCLCPP_INFO(get_logger(), "map_server active publishing static map from %s", map_file_.c_str());
  return true;
}

bool StackLifecycleManagerNode::ensure_static_map_loaded(std::string * err) {
  if (map_file_.empty()) {
    return true;
  }
  if (map_server_pid_ > 0 && map_server_yaml_ == map_file_) {
    const std::string st = query_lifecycle_state(resolve_lifecycle_node_fqn("map_server"));
    if (st == "active") {
      return true;
    }
  }
  stop_map_server();
  return start_map_server(err);
}

void StackLifecycleManagerNode::stop_slam_child() {
  if (slam_pid_ <= 0) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Stopping SLAM child pid=%d", slam_pid_);
  kill(slam_pid_, SIGINT);
  constexpr int max_wait_cycles = 50;
  for (int i = 0; i < max_wait_cycles; ++i) {
    int status = 0;
    const pid_t ret = waitpid(slam_pid_, &status, WNOHANG);
    if (ret == slam_pid_) {
      slam_pid_ = -1;
      return;
    }
    if (ret < 0) {
      slam_pid_ = -1;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  kill(slam_pid_, SIGKILL);
  waitpid(slam_pid_, nullptr, 0);
  slam_pid_ = -1;
}

bool StackLifecycleManagerNode::start_slam_child(const std::string & mode) {
  const std::string m = lower(mode);
  std::string executable;
  std::string params_file;
  std::string child_node_name;
  std::string slam_mode_param;
  std::string map_name;

  if (m == "mapping" || m == "map") {
    executable = "sync_slam_toolbox_node";
    params_file = mapper_params_file_;
    child_node_name = "mapping_worker";
    slam_mode_param = "mapping";
    map_name = mapping_map_topic_;
  } else if (m == "localize" || m == "localization" || m == "localizing") {
    executable = "localization_slam_toolbox_node";
    params_file = localization_params_file_;
    child_node_name = "localization_worker";
    slam_mode_param = "localization";
    map_name = localization_map_topic_;
    // Occupancy yaml is loaded by map_server onto static_map_topic_; slam must not claim /map.
    if (map_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "localize mode requires map_file (occupancy yaml) parameter");
      return false;
    }
    std::string map_err;
    if (!ensure_static_map_loaded(&map_err)) {
      RCLCPP_ERROR(get_logger(), "localize: static map load failed: %s", map_err.c_str());
      return false;
    }
  } else {
    return false;
  }

  if (params_file.empty()) {
    RCLCPP_ERROR(get_logger(), "SLAM params_file is empty for mode=%s", m.c_str());
    return false;
  }

  std::string exec_path;
  try {
    exec_path = ament_index_cpp::get_package_prefix("slam_toolbox") + "/lib/slam_toolbox/" + executable;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "slam_toolbox path: %s", e.what());
    return false;
  }
  if (access(exec_path.c_str(), X_OK) != 0) {
    RCLCPP_ERROR(get_logger(), "SLAM executable missing: %s", exec_path.c_str());
    return false;
  }

  std::vector<std::string> args = {
    exec_path,
    "--ros-args",
    "-r", "__node:=" + child_node_name,
    "-r", "__ns:=" + slam_child_namespace_,
    "--params-file", params_file,
    "-p", "use_sim_time:=" + std::string(use_sim_time_ ? "true" : "false"),
    "-p", "mode:=" + slam_mode_param,
    "-p", "map_frame:=" + map_frame_,
    "-p", "odom_frame:=" + odom_frame_,
    "-p", "base_frame:=" + base_frame_,
    "-p", "scan_topic:=" + scan_topic_,
    "-p", "map_name:=" + map_name,
  };
  if (m != "mapping" && m != "map") {
    const std::string posegraph = resolve_posegraph_stem();
    if (!posegraph.empty()) {
      args.push_back("-p");
      args.push_back("map_file_name:=" + posegraph);
      RCLCPP_INFO(get_logger(), "localize: loading slam posegraph stem %s", posegraph.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "localize: no .posegraph/.data beside map_file; slam will not deserialize occupancy yaml "
        "(static map is on %s via map_server)",
        static_map_topic_.c_str());
    }
    // task_manager / Web publish PoseWithCovarianceStamped on /<robot>/initial;
    // slam_toolbox localization listens to relative topic "initialpose".
    args.push_back("-r");
    args.push_back("initialpose:=" + std::string("/") + robot_id() + "/initial");
  }

  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (auto & item : args) {
    argv.push_back(const_cast<char *>(item.c_str()));
  }
  argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    RCLCPP_ERROR(get_logger(), "fork() failed");
    return false;
  }
  if (pid == 0) {
    execv(argv[0], argv.data());
    _exit(127);
  }

  slam_pid_ = pid;
  slam_mode_ = (m == "mapping" || m == "map") ? "mapping" : "localize";
  RCLCPP_INFO(
    get_logger(), "Started SLAM %s pid=%d node=%s ns=%s",
    slam_mode_.c_str(), slam_pid_, child_node_name.c_str(), slam_child_namespace_.c_str());
  return true;
}

bool StackLifecycleManagerNode::set_slam_mode(const std::string & mode, std::string * err) {
  const std::string m = lower(mode);
  if (m == "inactive" || m == "stop" || m == "none" || m == "off") {
    stop_slam_child();
    slam_mode_ = "inactive";
    return true;
  }
  if (m == slam_mode_ && slam_pid_ > 0) {
    return true;
  }
  stop_slam_child();
  if (!start_slam_child(m)) {
    slam_mode_ = "inactive";
    if (err) {
      *err = "failed to start slam mode " + m;
    }
    return false;
  }
  return true;
}

void StackLifecycleManagerNode::handle_transition(
  const std::shared_ptr<custom_msgs_srvs::srv::SetStackLifecycleTransition::Request> request,
  std::shared_ptr<custom_msgs_srvs::srv::SetStackLifecycleTransition::Response> response) {
  const std::string node_name = lower(strip(request->node_name));
  const std::string transition = lower(strip(request->transition));
  if (node_name.empty() || transition.empty()) {
    response->success = false;
    response->message = "node_name and transition are required";
    response->current_state = "";
    return;
  }

  if (node_name == "slam") {
    std::string err;
    response->success = set_slam_mode(transition, &err);
    response->current_state = slam_mode_;
    response->message = response->success ? "ok" : err;
    publish_stack_lifecycle();
    return;
  }

  const std::string fqn = resolve_lifecycle_node_fqn(node_name);
  std::string err;
  response->success = call_lifecycle_transition(fqn, transition, &err);
  response->current_state = query_lifecycle_state(fqn);
  response->message = response->success ? "ok" : err;
  publish_stack_lifecycle();
}

void StackLifecycleManagerNode::publish_stack_lifecycle() {
  try {
    custom_msgs_srvs::msg::StackLifecycle msg;
    msg.header.stamp = now();
    msg.robot_id = robot_id();

    custom_msgs_srvs::msg::StackComponentState slam;
    slam.name = "slam";
    if (slam_pid_ > 0) {
      slam.state = slam_mode_;
    } else {
      slam.state = "inactive";
    }
    msg.components.push_back(slam);

    for (const auto & short_name : tracked_lifecycle_nodes_) {
      custom_msgs_srvs::msg::StackComponentState comp;
      comp.name = short_name;
      comp.state = query_lifecycle_state(resolve_lifecycle_node_fqn(short_name));
      msg.components.push_back(comp);
    }

    stack_pub_->publish(msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "publish_stack_lifecycle failed: %s", e.what());
  }
}

}  // namespace manager

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manager::StackLifecycleManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4u);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
