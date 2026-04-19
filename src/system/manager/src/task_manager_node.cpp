#include "manager/task_manager_node.hpp"

#include <chrono>
#include <functional>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

namespace manager {

TaskManagerNode::TaskManagerNode()
: rclcpp::Node("task_manager") {
  hb_client_ = create_client<custom_msgs_srvs::srv::SetHeartbeatParams>("set_heartbeat_params");
  srv_ = create_service<custom_msgs_srvs::srv::SetRobotTask>(
    "set_robot_task",
    std::bind(&TaskManagerNode::on_set_task, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(
    get_logger(),
    "task_manager: ~/set_robot_task (uint8 TASK_STATUS_*) -> ~/set_heartbeat_params");
}

void TaskManagerNode::set_self(std::shared_ptr<TaskManagerNode> self) {
  self_ = std::move(self);
}

void TaskManagerNode::on_set_task(
  const std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Request> req,
  std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Response> res) {
  const uint8_t t = req->task_status;
  if (t > custom_msgs_srvs::msg::RobotStatus::TASK_STATUS_PATROLLING) {
    res->success = false;
    res->message = "task_status out of range (use RobotStatus.TASK_STATUS_*)";
    return;
  }
  if (!hb_client_->wait_for_service(std::chrono::seconds(2))) {
    res->success = false;
    res->message = "set_heartbeat_params not available";
    return;
  }
  auto hb_req = std::make_shared<custom_msgs_srvs::srv::SetHeartbeatParams::Request>();
  hb_req->robot_status =
    custom_msgs_srvs::srv::SetHeartbeatParams::Request::ROBOT_STATUS_LEAVE_UNCHANGED;
  hb_req->task_status = t;
  auto future = hb_client_->async_send_request(hb_req);
  if (!self_) {
    res->success = false;
    res->message = "task_manager not fully initialized";
    return;
  }
  if (rclcpp::spin_until_future_complete(self_, future, std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    res->success = false;
    res->message = "heartbeat call timeout";
    return;
  }
  const auto & hb_res = future.get();
  res->success = hb_res->success;
  res->message = hb_res->message;
}

}  // namespace manager

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manager::TaskManagerNode>();
  node->set_self(node);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2u);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
