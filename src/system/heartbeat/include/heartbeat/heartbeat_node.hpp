#ifndef HEARTBEAT__HEARTBEAT_NODE_HPP_
#define HEARTBEAT__HEARTBEAT_NODE_HPP_

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace heartbeat {

class HeartbeatNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit HeartbeatNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  std::string effective_robot_name() const;
  bool slam_mapping_node_present();
  static uint8_t clamp_robot_status(int v);
  static uint8_t clamp_task_status(int v);
  uint8_t resolved_robot_status_value();
  uint8_t resolved_task_status_value();
  void recreate_timer_locked();
  void tick();
  void on_set_params(
    const std::shared_ptr<custom_msgs_srvs::srv::SetHeartbeatParams::Request> request,
    std::shared_ptr<custom_msgs_srvs::srv::SetHeartbeatParams::Response> response);
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  static double publish_rate_hz_from_parameter(const rclcpp::Parameter & p, double default_hz);

  std::mutex mutex_;
  rclcpp_lifecycle::LifecyclePublisher<custom_msgs_srvs::msg::RobotStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<custom_msgs_srvs::srv::SetHeartbeatParams>::SharedPtr srv_;
  /// Last publish_rate (Hz) for which we logged timer recreation; NaN = never logged this session.
  double last_logged_timer_hz_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace heartbeat

#endif  // HEARTBEAT__HEARTBEAT_NODE_HPP_
