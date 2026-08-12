#ifndef MANAGER__HEALTH_MONITOR_NODE_HPP_
#define MANAGER__HEALTH_MONITOR_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "custom_msgs_srvs/msg/stack_lifecycle.hpp"
#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "manager/node_ping_checker.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manager {

class HealthMonitorNode : public rclcpp::Node {
public:
  HealthMonitorNode();
  void set_self(std::shared_ptr<HealthMonitorNode> self);
  /// Call after ``exec.spin()`` returns; pass the same executor so pending service work can drain.
  void best_effort_shutdown_after_spin(rclcpp::Executor * exec);

private:
  enum class Phase {
    Initializing,
    Localizing,
    LocalizationLost,
    Ready,
    ShutdownSent,
  };

  /// ``pump_exec``: when non-null, ``spin_some`` is used while waiting (e.g. after main ``spin`` returns).
  bool call_set_params(
    const std::string & robot_status, const std::string & current_map,
    rclcpp::Executor * pump_exec = nullptr);
  bool required_satisfied();
  void reset_map_baseline();
  void on_poll();
  void on_robot_status(const custom_msgs_srvs::msg::RobotStatus::SharedPtr msg);
  void on_stack_lifecycle(const custom_msgs_srvs::msg::StackLifecycle::SharedPtr msg);
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  std::recursive_mutex mtx_;
  Phase phase_{Phase::Initializing};
  std::vector<std::string> required_nodes_;
  std::vector<std::string> ping_nodes_;
  std::string pose_topic_;
  double cov_max_{0.45};
  bool have_map_baseline_{false};
  bool localization_module_active_{false};
  bool initial_ping_transition_done_{false};
  std::string map_baseline_;

  rclcpp::CallbackGroup::SharedPtr hb_client_cb_group_;
  rclcpp::Client<custom_msgs_srvs::srv::SetHeartbeatParams>::SharedPtr hb_client_;
  rclcpp::Subscription<custom_msgs_srvs::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<custom_msgs_srvs::msg::StackLifecycle>::SharedPtr stack_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  std::unique_ptr<NodePingChecker> node_ping_checker_;
  std::shared_ptr<HealthMonitorNode> self_;
};

}  // namespace manager

#endif  // MANAGER__HEALTH_MONITOR_NODE_HPP_
