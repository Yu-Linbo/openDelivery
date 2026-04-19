#ifndef MANAGER__HEALTH_MONITOR_NODE_HPP_
#define MANAGER__HEALTH_MONITOR_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manager {

class HealthMonitorNode : public rclcpp::Node {
public:
  HealthMonitorNode();
  void set_self(std::shared_ptr<HealthMonitorNode> self);
  void best_effort_shutdown_after_spin();

private:
  enum class Phase {
    Initializing,
    Localizing,
    LocalizationLost,
    Ready,
    ShutdownSent,
  };

  bool call_set_params(uint8_t robot_status, const std::string & current_map);
  bool required_satisfied();
  void reset_map_baseline();
  void on_poll();
  void on_robot_status(const custom_msgs_srvs::msg::RobotStatus::SharedPtr msg);
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  std::recursive_mutex mtx_;
  Phase phase_{Phase::Initializing};
  std::vector<std::string> required_nodes_;
  std::string pose_topic_;
  double cov_max_{0.45};
  bool allow_ready_from_localizing_{true};
  bool have_map_baseline_{false};
  std::string map_baseline_;

  rclcpp::Client<custom_msgs_srvs::srv::SetHeartbeatParams>::SharedPtr hb_client_;
  rclcpp::Subscription<custom_msgs_srvs::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  std::shared_ptr<HealthMonitorNode> self_;
};

}  // namespace manager

#endif  // MANAGER__HEALTH_MONITOR_NODE_HPP_
