#include "manager/health_monitor_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <thread>

#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

namespace {

std::string strip(const std::string & s) {
  const char * ws = " \t\n\r";
  const auto a = s.find_first_not_of(ws);
  if (a == std::string::npos) {
    return {};
  }
  const auto b = s.find_last_not_of(ws);
  return s.substr(a, b - a + 1);
}

std::string norm_ns(const std::string & raw) {
  if (raw.empty() || raw == "/") {
    return "/";
  }
  if (raw.front() != '/') {
    return std::string("/") + raw;
  }
  return raw;
}

bool cov_localized(const geometry_msgs::msg::PoseWithCovariance & p, double max_xy) {
  if (p.covariance.size() < 36) {
    return false;
  }
  const double cx = p.covariance[0];
  const double cy = p.covariance[7];
  if (!(cx >= 0.0 && cy >= 0.0)) {
    return false;
  }
  return cx <= max_xy && cy <= max_xy;
}

}  // namespace

namespace manager {

HealthMonitorNode::HealthMonitorNode()
: rclcpp::Node("health_monitor") {
  declare_parameter<std::vector<std::string>>(
    "required_nodes", std::vector<std::string>{"heartbeat"});
  declare_parameter<std::string>("localization_pose_topic", "amcl_pose");
  declare_parameter<double>("position_covariance_max", 0.45);
  declare_parameter<bool>("allow_ready_from_localizing", true);
  declare_parameter<double>("poll_period_sec", 1.0);

  required_nodes_ = get_parameter("required_nodes").as_string_array();
  pose_topic_ = strip(get_parameter("localization_pose_topic").as_string());
  cov_max_ = get_parameter("position_covariance_max").as_double();
  allow_ready_from_localizing_ = get_parameter("allow_ready_from_localizing").as_bool();
  const double period = std::max(0.2, get_parameter("poll_period_sec").as_double());

  hb_client_ = create_client<custom_msgs_srvs::srv::SetHeartbeatParams>("set_heartbeat_params");
  status_sub_ = create_subscription<custom_msgs_srvs::msg::RobotStatus>(
    "robot_status",
    rclcpp::QoS(10),
    std::bind(&HealthMonitorNode::on_robot_status, this, std::placeholders::_1));

  if (!pose_topic_.empty()) {
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_,
      rclcpp::QoS(10),
      std::bind(&HealthMonitorNode::on_pose, this, std::placeholders::_1));
  }

  poll_timer_ = create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&HealthMonitorNode::on_poll, this));

  RCLCPP_INFO(
    get_logger(),
    "health_monitor: nodes=%zu pose_topic=%s allow_ready_from_localizing=%s",
    required_nodes_.size(),
    pose_topic_.empty() ? "(disabled)" : pose_topic_.c_str(),
    allow_ready_from_localizing_ ? "true" : "false");
}

void HealthMonitorNode::set_self(std::shared_ptr<HealthMonitorNode> self) {
  self_ = std::move(self);
}

void HealthMonitorNode::best_effort_shutdown_after_spin() {
  {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    if (phase_ == Phase::ShutdownSent) {
      return;
    }
    phase_ = Phase::ShutdownSent;
  }
  (void)call_set_params(custom_msgs_srvs::msg::RobotStatus::ROBOT_STATUS_SHUTDOWN, "");
  if (!self_) {
    return;
  }
  for (int i = 0; i < 30 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(self_);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

bool HealthMonitorNode::call_set_params(uint8_t robot_status, const std::string & current_map) {
  if (!self_) {
    return false;
  }
  if (!hb_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "set_heartbeat_params not available");
    return false;
  }
  auto req = std::make_shared<custom_msgs_srvs::srv::SetHeartbeatParams::Request>();
  req->robot_status = robot_status;
  req->task_status = custom_msgs_srvs::srv::SetHeartbeatParams::Request::TASK_STATUS_LEAVE_UNCHANGED;
  if (!current_map.empty()) {
    req->current_map = current_map;
  }
  auto fut = hb_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(self_, fut, std::chrono::seconds(3)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }
  return fut.get()->success;
}

bool HealthMonitorNode::required_satisfied() {
  const std::string want = norm_ns(get_namespace());
  std::vector<std::pair<std::string, std::string>> pairs;
  try {
    pairs = get_node_graph_interface()->get_node_names_and_namespaces();
  } catch (const std::exception &) {
    return false;
  }
  for (const auto & need : required_nodes_) {
    bool ok = false;
    for (const auto & pr : pairs) {
      if (pr.first == need && norm_ns(pr.second) == want) {
        ok = true;
        break;
      }
    }
    if (!ok) {
      return false;
    }
  }
  return true;
}

void HealthMonitorNode::reset_map_baseline() {
  have_map_baseline_ = false;
  map_baseline_.clear();
}

void HealthMonitorNode::on_poll() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (phase_ == Phase::ShutdownSent) {
    return;
  }
  if (phase_ == Phase::Initializing) {
    if (required_satisfied()) {
      if (call_set_params(custom_msgs_srvs::msg::RobotStatus::ROBOT_STATUS_LOCALIZING, "")) {
        phase_ = Phase::Localizing;
        reset_map_baseline();
        RCLCPP_INFO(get_logger(), "robot_status -> localizing (required nodes up)");
      }
    }
  }
}

void HealthMonitorNode::on_robot_status(const custom_msgs_srvs::msg::RobotStatus::SharedPtr msg) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (phase_ != Phase::Localizing) {
    return;
  }
  const std::string m = strip(msg->current_map);
  if (m.empty()) {
    return;
  }
  if (!have_map_baseline_) {
    map_baseline_ = m;
    have_map_baseline_ = true;
    return;
  }
  if (m != map_baseline_) {
    if (call_set_params(custom_msgs_srvs::msg::RobotStatus::ROBOT_STATUS_LOCALIZATION_LOST, m)) {
      phase_ = Phase::LocalizationLost;
      map_baseline_ = m;
      RCLCPP_INFO(get_logger(), "robot_status -> localization_lost (map change)");
    }
  }
}

void HealthMonitorNode::on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (!cov_localized(msg->pose, cov_max_)) {
    return;
  }
  if (phase_ == Phase::LocalizationLost) {
    if (call_set_params(custom_msgs_srvs::msg::RobotStatus::ROBOT_STATUS_READY, "")) {
      phase_ = Phase::Ready;
      RCLCPP_INFO(get_logger(), "robot_status -> ready (localization ok)");
    }
    return;
  }
  if (phase_ == Phase::Localizing && allow_ready_from_localizing_) {
    if (call_set_params(custom_msgs_srvs::msg::RobotStatus::ROBOT_STATUS_READY, "")) {
      phase_ = Phase::Ready;
      RCLCPP_INFO(get_logger(), "robot_status -> ready (from localizing, covariance ok)");
    }
  }
}

}  // namespace manager

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manager::HealthMonitorNode>();
  node->set_self(node);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3u);
  exec.add_node(node);
  exec.spin();
  node->best_effort_shutdown_after_spin();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
