#ifndef HEARTBEAT__HEARTBEAT_NODE_HPP_
#define HEARTBEAT__HEARTBEAT_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace heartbeat {

class HeartbeatNode : public rclcpp::Node {
public:
  explicit HeartbeatNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::string effective_robot_name() const;
  bool slam_mapping_node_present();
  std::string resolved_robot_status_string();
  void recreate_io();
  void tick();
  void on_set_params(
    const std::shared_ptr<custom_msgs_srvs::srv::SetHeartbeatParams::Request> request,
    std::shared_ptr<custom_msgs_srvs::srv::SetHeartbeatParams::Response> response);

  static double publish_rate_hz_from_parameter(const rclcpp::Parameter & p, double default_hz);

  std::mutex mutex_;
  rclcpp::Publisher<custom_msgs_srvs::msg::RobotStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<custom_msgs_srvs::srv::SetHeartbeatParams>::SharedPtr srv_;
};

}  // namespace heartbeat

#endif  // HEARTBEAT__HEARTBEAT_NODE_HPP_
