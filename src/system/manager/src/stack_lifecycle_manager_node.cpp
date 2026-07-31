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
  declare_parameter<std::string>("static_map_topic", "");
  declare_parameter<std::string>("slam_child_namespace", "");
  declare_parameter<std::string>("initial_slam_mode", "inactive");
  declare_parameter<std::string>("robot_id", "");
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
  static_map_topic_ = get_parameter("static_map_topic").as_string();
  slam_child_namespace_ = get_parameter("slam_child_namespace").as_string();
  tracked_lifecycle_nodes_ = get_parameter("tracked_lifecycle_nodes").as_string_array();
  robot_id_ = strip(get_parameter("robot_id").as_string());

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
  if (static_map_topic_.empty()) {
    static_map_topic_ = std::string("/") + rid + "/map";
  }
  if (slam_child_namespace_.empty()) {
    slam_child_namespace_ = std::string("/") + rid + "/slam";
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
  initial_slam_mode_ = lower(get_parameter("initial_slam_mode").as_string());
  // Lifecycle service responses require this node's executor to be spinning.
  // Starting map_server/SLAM in the constructor deadlocks until timeout because
  // the node has not been added to the executor yet.
  initial_start_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&StackLifecycleManagerNode::start_initial_modules, this));

  RCLCPP_INFO(
    get_logger(),
    "slam lifecycle manager ready (robot=%s mode=%s static_map=%s mapping=%s)",
    rid.c_str(), slam_mode_.c_str(), static_map_topic_.c_str(), mapping_map_topic_.c_str());
}

void StackLifecycleManagerNode::start_initial_modules() {
  if (initial_start_timer_) {
    initial_start_timer_->cancel();
  }
  // Keep the saved static floor map available in both mapping and localization modes.
  if (!map_file_.empty()) {
    std::string map_err;
    if (!ensure_static_map_loaded(&map_err)) {
      RCLCPP_WARN(get_logger(), "static map load failed: %s", map_err.c_str());
    }
  }
  if (!initial_slam_mode_.empty() && initial_slam_mode_ != "inactive") {
    std::string err;
    if (!set_slam_mode(initial_slam_mode_, &err)) {
      RCLCPP_WARN(
        get_logger(), "initial_slam_mode=%s failed: %s",
        initial_slam_mode_.c_str(), err.c_str());
    }
  }
  initialization_complete_.store(true);
  publish_stack_lifecycle();
}

StackLifecycleManagerNode::~StackLifecycleManagerNode() {
  stop_slam_child();
  stop_map_server();
}

std::string StackLifecycleManagerNode::robot_id() const {
  if (!robot_id_.empty()) {
    return robot_id_;
  }
  const std::string leaf = leaf_namespace(get_namespace());
  return leaf.empty() ? std::string("robot") : leaf;
}

std::string StackLifecycleManagerNode::slam_node_fqn(const std::string & leaf) const {
  return std::string("/") + robot_id() + "/slam/" + leaf;
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

rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
StackLifecycleManagerNode::get_state_client(const std::string & node_fqn) {
  std::lock_guard<std::mutex> lock(lifecycle_clients_mutex_);
  auto it = get_state_clients_.find(node_fqn);
  if (it != get_state_clients_.end()) {
    return it->second;
  }
  auto client = create_client<lifecycle_msgs::srv::GetState>(
    node_fqn + "/get_state", rmw_qos_profile_services_default, client_cb_group_);
  get_state_clients_[node_fqn] = client;
  return client;
}

rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr
StackLifecycleManagerNode::change_state_client(const std::string & node_fqn) {
  std::lock_guard<std::mutex> lock(lifecycle_clients_mutex_);
  auto it = change_state_clients_.find(node_fqn);
  if (it != change_state_clients_.end()) {
    return it->second;
  }
  auto client = create_client<lifecycle_msgs::srv::ChangeState>(
    node_fqn + "/change_state", rmw_qos_profile_services_default, client_cb_group_);
  change_state_clients_[node_fqn] = client;
  return client;
}

std::string StackLifecycleManagerNode::query_lifecycle_state(const std::string & node_fqn) {
  try {
    auto client = get_state_client(node_fqn);
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
  auto client = change_state_client(node_fqn);
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

bool StackLifecycleManagerNode::navigation_active() {
  if (!navigation_active_client_) {
    navigation_active_client_ = create_client<std_srvs::srv::Trigger>(
      resolve_lifecycle_node_fqn("lifecycle_manager_navigation") + "/is_active",
      rmw_qos_profile_services_default,
      client_cb_group_);
  }
  if (!navigation_active_client_->wait_for_service(std::chrono::milliseconds(100))) {
    return false;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = navigation_active_client_->async_send_request(request);
  if (!wait_future(future, std::chrono::milliseconds(300))) {
    return false;
  }
  const auto response = future.get();
  return response && response->success;
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
  if (slam_pid_ > 0) {
    RCLCPP_INFO(get_logger(), "Stopping GMapping pid=%d", slam_pid_);
    kill(slam_pid_, SIGINT);
    for (int i = 0; i < 50; ++i) {
      if (waitpid(slam_pid_, nullptr, WNOHANG) == slam_pid_) {
        slam_pid_ = -1;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (slam_pid_ > 0) {
      kill(slam_pid_, SIGKILL);
      waitpid(slam_pid_, nullptr, 0);
      slam_pid_ = -1;
    }
  }
  const std::string amcl = slam_node_fqn("localizing");
  if (query_lifecycle_state(amcl) == "active") {
    std::string err;
    if (!call_lifecycle_transition(amcl, "deactivate", &err)) {
      RCLCPP_ERROR(get_logger(), "failed to deactivate AMCL: %s", err.c_str());
    }
  }
}

bool StackLifecycleManagerNode::start_slam_child(const std::string & mode) {
  const std::string m = lower(mode);
  if (m == "mapping" || m == "map") {
    std::string exec_path;
    try {
      exec_path = ament_index_cpp::get_package_prefix("slam_gmapping") +
        "/lib/slam_gmapping/slam_gmapping";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "resolve slam_gmapping: %s", e.what());
      return false;
    }
    const std::string rid = robot_id();
    std::vector<std::string> args = {
      exec_path, "--ros-args",
      "-r", "__node:=mapping", "-r", "__ns:=/" + rid + "/slam",
      "-r", "scan:=" + scan_topic_, "-r", "map:=" + mapping_map_topic_,
      "-p", "use_sim_time:=" + std::string(use_sim_time_ ? "true" : "false"),
      "-p", "base_frame:=" + base_frame_, "-p", "odom_frame:=" + odom_frame_,
      "-p", "map_frame:=" + map_frame_};
    if (!mapper_params_file_.empty()) {
      args.insert(args.end(), {"--params-file", mapper_params_file_});
    }
    std::vector<char *> argv;
    for (auto & arg : args) { argv.push_back(const_cast<char *>(arg.c_str())); }
    argv.push_back(nullptr);
    const pid_t pid = fork();
    if (pid < 0) { return false; }
    if (pid == 0) { execv(argv[0], argv.data()); _exit(127); }
    slam_pid_ = pid;
    slam_mode_ = "mapping";
    RCLCPP_INFO(get_logger(), "GMapping active pid=%d", slam_pid_);
    return true;
  }
  if (m != "localize" && m != "localization" && m != "localizing") {
    return false;
  }
  if (map_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "AMCL localization requires map_file");
    return false;
  }
  std::string err;
  if (!ensure_static_map_loaded(&err)) {
    RCLCPP_ERROR(get_logger(), "AMCL map load failed: %s", err.c_str());
    return false;
  }
  const std::string amcl = slam_node_fqn("localizing");
  std::string state = query_lifecycle_state(amcl);
  if (state == "unconfigured") {
    if (!call_lifecycle_transition(amcl, "configure", &err)) {
      RCLCPP_ERROR(get_logger(), "configure AMCL: %s", err.c_str());
      return false;
    }
    state = "inactive";
  }
  if (state != "active" && !call_lifecycle_transition(amcl, "activate", &err)) {
    RCLCPP_ERROR(get_logger(), "activate AMCL: %s", err.c_str());
    return false;
  }
  slam_mode_ = "localize";
  RCLCPP_INFO(get_logger(), "AMCL localization active");
  return true;
}

bool StackLifecycleManagerNode::set_slam_mode(const std::string & mode, std::string * err) {
  const std::string m = lower(mode);
  if (m == "inactive" || m == "stop" || m == "none" || m == "off") {
    stop_slam_child();
    slam_mode_ = "inactive";
    return true;
  }
  if (m == slam_mode_ && ((m == "mapping" && slam_pid_ > 0) ||
      (m != "mapping" && query_lifecycle_state(slam_node_fqn("localizing")) == "active"))) {
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
  if (!initialization_complete_.load()) {
    return;
  }
  try {
    custom_msgs_srvs::msg::StackLifecycle msg;
    msg.header.stamp = now();
    msg.robot_id = robot_id();

    custom_msgs_srvs::msg::StackComponentState localization;
    localization.name = "localization";
    const bool localization_running =
      query_lifecycle_state(slam_node_fqn("localizing")) == "active";
    localization.state = localization_running ? "active" : "inactive";
    msg.components.push_back(localization);

    custom_msgs_srvs::msg::StackComponentState navigation;
    navigation.name = "navigation";
    navigation.state = navigation_active() ? "active" : "inactive";
    msg.components.push_back(navigation);

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
