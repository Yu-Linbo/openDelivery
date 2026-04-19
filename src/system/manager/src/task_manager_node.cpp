#include "manager/task_manager_node.hpp"

#include <cmath>
#include <chrono>
#include <functional>
#include <future>
#include <thread>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

namespace {

std::string strip(const std::string & s) {
  const char * ws = " \t\n\r";
  const auto a = s.find_first_not_of(ws);
  if (a == std::string::npos) {
    return {};
  }
  const auto b = s.find_last_not_of(ws);
  return s.substr(a, b - a + 1);
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
  const auto p = ns.find_last_of('/');
  if (p == std::string::npos) {
    return ns;
  }
  return ns.substr(p + 1);
}

template<typename FutureT>
bool wait_client_future_no_spin(const FutureT & fut, std::chrono::seconds timeout) {
  const auto step = std::chrono::milliseconds(20);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (fut.wait_for(step) == std::future_status::ready) {
      return true;
    }
    std::this_thread::sleep_for(step);
  }
  return fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

}  // namespace

namespace manager {

using RobotStatusMsg = custom_msgs_srvs::msg::RobotStatus;
using Srv = custom_msgs_srvs::srv::SetHeartbeatParams;

TaskManagerNode::TaskManagerNode()
: rclcpp::Node("task_manager") {
  declare_parameter<std::string>("initial_pose_topic", "initial");
  initial_pose_topic_ = strip(get_parameter("initial_pose_topic").as_string());
  if (initial_pose_topic_.empty()) {
    initial_pose_topic_ = "initial";
  }

  hb_client_ = create_client<custom_msgs_srvs::srv::SetHeartbeatParams>("set_heartbeat_params");
  srv_ = create_service<custom_msgs_srvs::srv::SetRobotTask>(
    "set_robot_task",
    std::bind(&TaskManagerNode::on_set_task, this, std::placeholders::_1, std::placeholders::_2));

  status_gate_sub_ = create_subscription<RobotStatusMsg>(
    "robot_status",
    rclcpp::QoS(10),
    std::bind(&TaskManagerNode::on_robot_status_for_gate, this, std::placeholders::_1));

  localize_sub_ = create_subscription<custom_msgs_srvs::msg::LocalizeNavCommand>(
    "localize_nav_command",
    rclcpp::QoS(10),
    std::bind(&TaskManagerNode::on_localize_nav, this, std::placeholders::_1));

  initial_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initial_pose_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(
    get_logger(),
    "task_manager: ~/set_robot_task + sub localize_nav_command -> ~/set_heartbeat_params; "
    "pub initial on %s",
    initial_pose_topic_.c_str());
}

void TaskManagerNode::set_self(std::shared_ptr<TaskManagerNode> self) {
  self_ = std::move(self);
}

void TaskManagerNode::on_robot_status_for_gate(const RobotStatusMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  last_robot_status_ = msg->robot_status;
  last_task_status_ = msg->task_status;
}

bool TaskManagerNode::send_heartbeat(
  uint8_t robot_status,
  uint8_t task_status,
  const std::string & current_map) {
  if (!hb_client_->wait_for_service(std::chrono::milliseconds(800))) {
    return false;
  }
  auto req = std::make_shared<Srv::Request>();
  req->robot_name = "";
  req->robot_status = robot_status;
  req->task_status = task_status;
  req->current_map = current_map;
  req->rate_hz = 0.0;
  auto fut = hb_client_->async_send_request(req);
  if (!self_) {
    return false;
  }
  if (!wait_client_future_no_spin(fut, std::chrono::seconds(8))) {
    return false;
  }
  const auto res = fut.get();
  return res && res->success;
}

void TaskManagerNode::on_set_task(
  const std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Request> req,
  std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Response> res) {
  const uint8_t t = req->task_status;
  if (t > RobotStatusMsg::TASK_STATUS_PATROLLING) {
    res->success = false;
    res->message = "task_status out of range (use RobotStatus.TASK_STATUS_*)";
    return;
  }
  if (!hb_client_->wait_for_service(std::chrono::seconds(2))) {
    res->success = false;
    res->message = "set_heartbeat_params not available";
    return;
  }
  auto hb_req = std::make_shared<Srv::Request>();
  const uint8_t rs = req->robot_status;
  if (rs == custom_msgs_srvs::srv::SetRobotTask::Request::ROBOT_STATUS_LEAVE_UNCHANGED) {
    hb_req->robot_status = Srv::Request::ROBOT_STATUS_LEAVE_UNCHANGED;
  } else if (rs <= RobotStatusMsg::ROBOT_STATUS_SHUTDOWN) {
    hb_req->robot_status = rs;
  } else {
    res->success = false;
    res->message = "robot_status out of range (0..4 or 255 leave unchanged)";
    return;
  }
  hb_req->task_status = t;
  hb_req->robot_name = "";
  hb_req->rate_hz = 0.0;
  auto future = hb_client_->async_send_request(hb_req);
  if (!self_) {
    res->success = false;
    res->message = "task_manager not fully initialized";
    return;
  }
  if (!wait_client_future_no_spin(future, std::chrono::seconds(8))) {
    res->success = false;
    res->message = "heartbeat call timeout";
    return;
  }
  const auto & hb_res = future.get();
  res->success = hb_res->success;
  res->message = hb_res->message;
}

void TaskManagerNode::on_localize_nav(
  const custom_msgs_srvs::msg::LocalizeNavCommand::SharedPtr msg) {
  const std::string want = leaf_namespace_id(get_namespace());
  const std::string rid = strip(msg->robot_id);
  if (rid.empty() || rid != want) {
    RCLCPP_WARN(
      get_logger(),
      "localize_nav_command ignored: robot_id %s != namespace id %s",
      rid.c_str(), want.c_str());
    return;
  }

  uint8_t rs = 0;
  uint8_t ts = 0;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    rs = last_robot_status_;
    ts = last_task_status_;
  }

  if (ts == RobotStatusMsg::TASK_STATUS_MAPPING) {
    RCLCPP_WARN(get_logger(), "localize_nav_command ignored: task_status is mapping");
    return;
  }
  if (rs == RobotStatusMsg::ROBOT_STATUS_SHUTDOWN) {
    RCLCPP_WARN(get_logger(), "localize_nav_command ignored: robot_status is shutdown");
    return;
  }
  if (rs != RobotStatusMsg::ROBOT_STATUS_INITIALIZING &&
    rs != RobotStatusMsg::ROBOT_STATUS_LOCALIZING &&
    rs != RobotStatusMsg::ROBOT_STATUS_READY)
  {
    RCLCPP_WARN(
      get_logger(),
      "localize_nav_command ignored: robot_status=%u not in {init,localizing,ready}",
      static_cast<unsigned>(rs));
    return;
  }

  const std::string map = strip(msg->map_name);
  if (map.empty() && !msg->set_initial_pose) {
    RCLCPP_WARN(get_logger(), "localize_nav_command ignored: empty map_name and no pose");
    return;
  }

  // 1) task idle (mapping off)
  if (!send_heartbeat(Srv::Request::ROBOT_STATUS_LEAVE_UNCHANGED, RobotStatusMsg::TASK_STATUS_IDLE, "")) {
    RCLCPP_ERROR(get_logger(), "localize_nav: failed to set task idle");
    return;
  }
  // 2) current_map (切图)
  if (!map.empty()) {
    if (!send_heartbeat(
        Srv::Request::ROBOT_STATUS_LEAVE_UNCHANGED,
        Srv::Request::TASK_STATUS_LEAVE_UNCHANGED,
        map))
    {
      RCLCPP_ERROR(get_logger(), "localize_nav: failed to set current_map");
      return;
    }
  }
  // 3) localization lost (health_monitor will drive -> ready from pose topic)
  if (!send_heartbeat(RobotStatusMsg::ROBOT_STATUS_LOCALIZATION_LOST, Srv::Request::TASK_STATUS_LEAVE_UNCHANGED, "")) {
    RCLCPP_ERROR(get_logger(), "localize_nav: failed to set robot_status localization_lost");
    return;
  }
  // 4) initial pose for AMCL / stack
  if (msg->set_initial_pose && initial_pub_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = msg->x;
    pose.pose.pose.position.y = msg->y;
    pose.pose.pose.position.z = 0.0;
    const double yaw = msg->yaw;
    pose.pose.pose.orientation.x = 0.0;
    pose.pose.pose.orientation.y = 0.0;
    pose.pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.pose.orientation.w = std::cos(yaw / 2.0);
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0] = 0.25;
    pose.pose.covariance[7] = 0.25;
    pose.pose.covariance[35] = 0.068;
    initial_pub_->publish(pose);
    RCLCPP_INFO(
      get_logger(),
      "localize_nav: published initial pose x=%.3f y=%.3f yaw=%.4f",
      msg->x, msg->y, yaw);
  }

  RCLCPP_INFO(get_logger(), "localize_nav_command applied for %s", rid.c_str());
}

}  // namespace manager

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manager::TaskManagerNode>();
  node->set_self(node);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4u);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
