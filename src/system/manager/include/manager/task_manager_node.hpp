#ifndef MANAGER__TASK_MANAGER_NODE_HPP_
#define MANAGER__TASK_MANAGER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "custom_msgs_srvs/msg/localize_nav_command.hpp"
#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "custom_msgs_srvs/srv/set_robot_task.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manager {

class TaskManagerNode : public rclcpp::Node {
public:
  TaskManagerNode();
  void set_self(std::shared_ptr<TaskManagerNode> self);

private:
  void on_set_task(
    const std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Request> req,
    std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Response> res);

  void on_robot_status_for_gate(const custom_msgs_srvs::msg::RobotStatus::SharedPtr msg);
  void on_localize_nav(const custom_msgs_srvs::msg::LocalizeNavCommand::SharedPtr msg);

  bool send_heartbeat(
    const std::string & robot_status,
    const std::string & task_status,
    const std::string & current_map);

  // Separate group so localize/set_task callbacks can wait on heartbeat without deadlock.
  rclcpp::CallbackGroup::SharedPtr hb_client_cb_group_;
  rclcpp::Client<custom_msgs_srvs::srv::SetHeartbeatParams>::SharedPtr hb_client_;
  rclcpp::Service<custom_msgs_srvs::srv::SetRobotTask>::SharedPtr srv_;
  rclcpp::Subscription<custom_msgs_srvs::msg::RobotStatus>::SharedPtr status_gate_sub_;
  rclcpp::Subscription<custom_msgs_srvs::msg::LocalizeNavCommand>::SharedPtr localize_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pub_;

  std::mutex mtx_;
  std::string last_robot_status_{"initializing"};
  std::string last_task_status_{"idle"};
  std::string initial_pose_topic_;

  std::shared_ptr<TaskManagerNode> self_;
};

}  // namespace manager

#endif  // MANAGER__TASK_MANAGER_NODE_HPP_
