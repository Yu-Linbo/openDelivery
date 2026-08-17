#ifndef MANAGER__TASK_MANAGER_NODE_HPP_
#define MANAGER__TASK_MANAGER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "custom_msgs_srvs/msg/elevator_command.hpp"
#include "custom_msgs_srvs/msg/elevator_info.hpp"
#include "custom_msgs_srvs/msg/elevator_status.hpp"
#include "custom_msgs_srvs/msg/localize_nav_command.hpp"
#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "custom_msgs_srvs/msg/task_command.hpp"
#include "custom_msgs_srvs/msg/task_info.hpp"
#include "custom_msgs_srvs/msg/task_status.hpp"
#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "custom_msgs_srvs/srv/set_robot_task.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manager {

class TaskManagerElevatorPlanTestPeer;

class TaskManagerNode : public rclcpp::Node {
public:
  explicit TaskManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void set_self(std::shared_ptr<TaskManagerNode> self);

private:
  friend class TaskManagerElevatorPlanTestPeer;
  void on_set_task(
    const std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Request> req,
    std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Response> res);

  void on_robot_status_for_gate(const custom_msgs_srvs::msg::RobotStatus::SharedPtr msg);
  void on_localize_nav(const custom_msgs_srvs::msg::LocalizeNavCommand::SharedPtr msg);
  void on_task_info(const custom_msgs_srvs::msg::TaskInfo::SharedPtr msg);
  void on_task_command(const custom_msgs_srvs::msg::TaskCommand::SharedPtr msg);
  void on_navigation_status(const custom_msgs_srvs::msg::TaskStatus::SharedPtr msg);
  void on_elevator_status(const custom_msgs_srvs::msg::ElevatorStatus::SharedPtr msg);
  void dispatch_current_work();
  void publish_task_status(const std::string & status, const std::string & message);

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
  rclcpp::Subscription<custom_msgs_srvs::msg::TaskInfo>::SharedPtr task_info_sub_;
  rclcpp::Subscription<custom_msgs_srvs::msg::TaskCommand>::SharedPtr task_command_sub_;
  rclcpp::Subscription<custom_msgs_srvs::msg::TaskStatus>::SharedPtr navigation_status_sub_;
  rclcpp::Publisher<custom_msgs_srvs::msg::TaskInfo>::SharedPtr navigation_info_pub_;
  rclcpp::Publisher<custom_msgs_srvs::msg::TaskCommand>::SharedPtr navigation_command_pub_;
  rclcpp::Publisher<custom_msgs_srvs::msg::TaskStatus>::SharedPtr task_status_pub_;
  rclcpp::Subscription<custom_msgs_srvs::msg::ElevatorStatus>::SharedPtr elevator_status_sub_;
  rclcpp::Publisher<custom_msgs_srvs::msg::ElevatorInfo>::SharedPtr elevator_info_pub_;
  rclcpp::Publisher<custom_msgs_srvs::msg::ElevatorCommand>::SharedPtr elevator_command_pub_;

  struct WorkItem {
    std::string kind;
    std::string label;
    custom_msgs_srvs::msg::TaskInfo navigation;
    custom_msgs_srvs::msg::ElevatorInfo elevator;
  };

  std::mutex mtx_;
  std::string last_robot_status_{"initializing"};
  std::string last_task_status_{"idle"};
  std::string last_current_map_;
  std::string initial_pose_topic_;
  std::string map_root_;
  custom_msgs_srvs::msg::TaskInfo active_task_;
  custom_msgs_srvs::msg::TaskStatus active_status_;
  bool has_active_task_{false};
  std::vector<WorkItem> work_items_;
  size_t current_work_index_{0};

  std::shared_ptr<TaskManagerNode> self_;
};

}  // namespace manager

#endif  // MANAGER__TASK_MANAGER_NODE_HPP_
