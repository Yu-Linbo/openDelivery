#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

using custom_msgs_srvs::srv::SetHeartbeatParams;

namespace {

std::string strip_robot_name_for_topic(std::string s) {
  const char * ws = " \t";
  auto start = s.find_first_not_of(ws);
  if (start == std::string::npos) {
    return "robot2";
  }
  auto end = s.find_last_not_of(ws);
  s = s.substr(start, end - start + 1);
  while (!s.empty() && s.front() == '/') {
    s.erase(0, 1);
  }
  while (!s.empty() && s.back() == '/') {
    s.pop_back();
  }
  return s.empty() ? "robot2" : s;
}

std::string strip_spaces(const std::string & s) {
  const char * ws = " \t";
  auto start = s.find_first_not_of(ws);
  if (start == std::string::npos) {
    return {};
  }
  auto end = s.find_last_not_of(ws);
  return s.substr(start, end - start + 1);
}

double publish_rate_hz_from_parameter(const rclcpp::Parameter & p, double default_hz) {
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

}  // namespace

class HeartbeatNode : public rclcpp::Node {
public:
  HeartbeatNode()
  : Node("robot_status_heartbeat") {
    declare_parameter<std::string>("robot_name", "robot2");
    declare_parameter<std::string>("current_map", "");
    declare_parameter<std::string>("robot_status", "normal");
    declare_parameter<double>("publish_rate", 2.0);

    recreate_io();

    srv_ = create_service<SetHeartbeatParams>(
      "set_heartbeat_params",
      std::bind(&HeartbeatNode::on_set_params, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "heartbeat ready; service set_heartbeat_params; topic from robot_name param");
  }

private:
  std::string robot_topic() const {
    const std::string rn = get_parameter("robot_name").as_string();
    return "/" + strip_robot_name_for_topic(rn) + "/robot_status";
  }

  void recreate_io() {
    std::lock_guard<std::mutex> lock(mutex_);
    timer_.reset();
    pub_.reset();

    const std::string topic = robot_topic();
    pub_ = create_publisher<custom_msgs_srvs::msg::RobotStatus>(topic, rclcpp::QoS(10));

    const double hz = publish_rate_hz_from_parameter(get_parameter("publish_rate"), 2.0);
    const auto period = rclcpp::Duration::from_seconds(1.0 / hz);
    timer_ = rclcpp::create_timer(
      this, get_clock(), period, std::bind(&HeartbeatNode::tick, this));

    RCLCPP_INFO(get_logger(), "publishing RobotStatus on %s at %.2f Hz", topic.c_str(), hz);
  }

  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pub_) {
      return;
    }
    custom_msgs_srvs::msg::RobotStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.robot_name = get_parameter("robot_name").as_string();
    msg.current_map = get_parameter("current_map").as_string();
    msg.robot_status = get_parameter("robot_status").as_string();
    pub_->publish(msg);
  }

  void on_set_params(
    const std::shared_ptr<SetHeartbeatParams::Request> request,
    std::shared_ptr<SetHeartbeatParams::Response> response) {
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

      recreate_io();
      response->success = true;
      response->message = "ok";
    } catch (const std::exception & e) {
      response->success = false;
      response->message = e.what();
    }
  }

  std::mutex mutex_;
  rclcpp::Publisher<custom_msgs_srvs::msg::RobotStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<SetHeartbeatParams>::SharedPtr srv_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatNode>());
  rclcpp::shutdown();
  return 0;
}
