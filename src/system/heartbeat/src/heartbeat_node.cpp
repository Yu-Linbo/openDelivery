#include "heartbeat/heartbeat_node.hpp"

#include <cctype>
#include <chrono>
#include <utility>
#include <vector>

#include "rclcpp/create_timer.hpp"

namespace {

std::string strip_spaces(const std::string & s) {
  const char * ws = " \t";
  auto start = s.find_first_not_of(ws);
  if (start == std::string::npos) {
    return {};
  }
  auto end = s.find_last_not_of(ws);
  return s.substr(start, end - start + 1);
}

std::string leaf_namespace_id(const std::string & raw_ns) {
  std::string ns = raw_ns;
  if (!ns.empty() && ns.front() == '/') {
    ns.erase(0, 1);
  }
  while (!ns.empty() && ns.back() == '/') {
    ns.pop_back();
  }
  if (ns.empty()) {
    return {};
  }
  const auto slash = ns.find_last_of('/');
  if (slash == std::string::npos) {
    return ns;
  }
  return ns.substr(slash + 1);
}

std::string fully_qualified_name(std::string ns, const std::string & name) {
  if (ns.empty() || ns == "/") {
    return std::string("/") + name;
  }
  if (ns.back() == '/') {
    ns.pop_back();
  }
  return ns + "/" + name;
}

bool parameter_indicates_simulation(const std::string & raw) {
  const std::string s = strip_spaces(raw);
  if (s.empty()) {
    return true;
  }
  std::string lower;
  lower.reserve(s.size());
  for (unsigned char ch : s) {
    lower.push_back(static_cast<char>(std::tolower(ch)));
  }
  return lower != "real";
}

}  // namespace

namespace heartbeat {

double HeartbeatNode::publish_rate_hz_from_parameter(
  const rclcpp::Parameter & p,
  double default_hz) {
  double rate = default_hz;
  try {
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        rate = p.as_double();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        rate = static_cast<double>(p.as_int());
        break;
      case rclcpp::ParameterType::PARAMETER_STRING: {
        const std::string s = p.as_string();
        if (!s.empty()) {
          rate = std::stod(s);
        }
        break;
      }
      default:
        break;
    }
  } catch (const std::exception &) {
    rate = default_hz;
  }
  if (rate < 0.5) {
    rate = 0.5;
  }
  return rate;
}

HeartbeatNode::HeartbeatNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("heartbeat", options) {
  declare_parameter<std::string>("robot_name", "");
  declare_parameter<std::string>("current_map", "");
  declare_parameter<std::string>("robot_status", "normal");
  declare_parameter<bool>("mapping_mode", false);
  declare_parameter<bool>("auto_mapping_status", true);
  declare_parameter<std::string>("sim_mode", "sim");
  declare_parameter<double>("publish_rate", 2.0);

  const std::string ns = get_namespace();
  if (ns.empty() || ns == "/" || ns == "~") {
    RCLCPP_WARN(
      get_logger(),
      "heartbeat running in the root namespace; launch with PushRosNamespace (e.g. "
      "ros2 launch heartbeat heartbeat.launch.py namespace:=robot2) so each instance has "
      "isolated topics and services");
  }

  srv_ = create_service<custom_msgs_srvs::srv::SetHeartbeatParams>(
    "set_heartbeat_params",
    std::bind(
      &HeartbeatNode::on_set_params, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "heartbeat lifecycle node ready: FQN=%s; use lifecycle transitions configure->activate before publishing",
    fully_qualified_name(get_namespace(), get_name()).c_str());
}

std::string HeartbeatNode::effective_robot_name() const {
  const std::string from_param = strip_spaces(get_parameter("robot_name").as_string());
  if (!from_param.empty()) {
    return from_param;
  }
  std::string leaf = leaf_namespace_id(get_namespace());
  return leaf.empty() ? std::string{"robot"} : leaf;
}

bool HeartbeatNode::slam_mapping_node_present() {
  std::string rn = effective_robot_name();
  while (!rn.empty() && rn.front() == '/') {
    rn.erase(0, 1);
  }
  if (rn.empty()) {
    return false;
  }
  const std::string expected_ns = std::string("/") + rn + "/slam_bringup";
  std::vector<std::pair<std::string, std::string>> pairs;
  try {
    pairs = get_node_graph_interface()->get_node_names_and_namespaces();
  } catch (const std::exception &) {
    return false;
  }
  for (const auto & pr : pairs) {
    if (pr.first == "mapping" && pr.second == expected_ns) {
      return true;
    }
  }
  return false;
}

std::string HeartbeatNode::resolved_robot_status_string() {
  std::string status = strip_spaces(get_parameter("robot_status").as_string());
  if (status.empty()) {
    status = "normal";
  }
  const bool manual_mapping = get_parameter("mapping_mode").as_bool();
  const bool auto_mapping =
    get_parameter("auto_mapping_status").as_bool() && slam_mapping_node_present();
  if (manual_mapping || auto_mapping) {
    return "mapping";
  }
  return status;
}

void HeartbeatNode::recreate_timer_locked() {
  timer_.reset();
  const double hz = publish_rate_hz_from_parameter(get_parameter("publish_rate"), 2.0);
  const auto period = rclcpp::Duration::from_seconds(1.0 / hz);
  timer_ = rclcpp::create_timer(
    this, get_clock(), period, std::bind(&HeartbeatNode::tick, this));

  const std::string fqn = fully_qualified_name(get_namespace(), get_name());
  const std::string ns = get_namespace();
  RCLCPP_INFO(
    get_logger(),
    "RobotStatus publisher relative name 'robot_status' (namespace '%s') -> effective topic under graph; "
    "node FQN %s at %.2f Hz",
    ns.c_str(),
    fqn.c_str(),
    hz);
}

void HeartbeatNode::tick() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pub_ || !pub_->is_activated()) {
    return;
  }
  custom_msgs_srvs::msg::RobotStatus msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.robot_name = effective_robot_name();
  msg.robot_status = resolved_robot_status_string();
  if (msg.robot_status == "mapping") {
    msg.current_map = msg.robot_name + "_mapping";
  } else {
    msg.current_map = get_parameter("current_map").as_string();
  }
  msg.is_simulation = parameter_indicates_simulation(get_parameter("sim_mode").as_string());
  pub_->publish(msg);
}

void HeartbeatNode::on_set_params(
  const std::shared_ptr<custom_msgs_srvs::srv::SetHeartbeatParams::Request> request,
  std::shared_ptr<custom_msgs_srvs::srv::SetHeartbeatParams::Response> response) {
  try {
    std::vector<rclcpp::Parameter> new_params;

    const std::string rn_in = strip_spaces(request->robot_name);
    if (!rn_in.empty()) {
      new_params.emplace_back("robot_name", rn_in);
    }
    if (request->current_map != "") {
      new_params.emplace_back("current_map", request->current_map);
    }
    if (request->robot_status != "") {
      new_params.emplace_back("robot_status", request->robot_status);
    }
    if (request->rate_hz > 0.0) {
      new_params.emplace_back("publish_rate", request->rate_hz);
    }

    if (!new_params.empty()) {
      auto results = set_parameters(new_params);
      for (const auto & r : results) {
        if (!r.successful) {
          response->success = false;
          response->message = r.reason.empty() ? "set_parameters rejected" : r.reason;
          return;
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (timer_) {
        recreate_timer_locked();
      }
    }
    response->success = true;
    response->message = "ok";
  } catch (const std::exception & e) {
    response->success = false;
    response->message = e.what();
  }
}

HeartbeatNode::CallbackReturn HeartbeatNode::on_configure(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(mutex_);
  pub_ = create_publisher<custom_msgs_srvs::msg::RobotStatus>("robot_status", rclcpp::QoS(10));
  recreate_timer_locked();
  RCLCPP_INFO(get_logger(), "heartbeat configured");
  return CallbackReturn::SUCCESS;
}

HeartbeatNode::CallbackReturn HeartbeatNode::on_activate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pub_) {
    RCLCPP_ERROR(get_logger(), "activate requested before configure");
    return CallbackReturn::FAILURE;
  }
  pub_->on_activate();
  RCLCPP_INFO(get_logger(), "heartbeat active");
  return CallbackReturn::SUCCESS;
}

HeartbeatNode::CallbackReturn HeartbeatNode::on_deactivate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pub_) {
    pub_->on_deactivate();
  }
  RCLCPP_INFO(get_logger(), "heartbeat inactive");
  return CallbackReturn::SUCCESS;
}

HeartbeatNode::CallbackReturn HeartbeatNode::on_cleanup(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(mutex_);
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(get_logger(), "heartbeat cleaned up");
  return CallbackReturn::SUCCESS;
}

HeartbeatNode::CallbackReturn HeartbeatNode::on_shutdown(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(mutex_);
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(get_logger(), "heartbeat shutdown");
  return CallbackReturn::SUCCESS;
}

}  // namespace heartbeat
