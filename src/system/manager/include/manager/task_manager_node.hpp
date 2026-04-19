#ifndef MANAGER__TASK_MANAGER_NODE_HPP_
#define MANAGER__TASK_MANAGER_NODE_HPP_

#include <memory>

#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "custom_msgs_srvs/srv/set_robot_task.hpp"
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

  rclcpp::Client<custom_msgs_srvs::srv::SetHeartbeatParams>::SharedPtr hb_client_;
  rclcpp::Service<custom_msgs_srvs::srv::SetRobotTask>::SharedPtr srv_;
  std::shared_ptr<TaskManagerNode> self_;
};

}  // namespace manager

#endif  // MANAGER__TASK_MANAGER_NODE_HPP_
