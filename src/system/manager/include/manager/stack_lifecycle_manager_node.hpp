#ifndef MANAGER__STACK_LIFECYCLE_MANAGER_NODE_HPP_
#define MANAGER__STACK_LIFECYCLE_MANAGER_NODE_HPP_

#include <map>
#include <string>
#include <vector>

#include "custom_msgs_srvs/msg/stack_lifecycle.hpp"
#include "custom_msgs_srvs/srv/set_stack_lifecycle_transition.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manager {

class StackLifecycleManagerNode : public rclcpp::Node {
public:
  StackLifecycleManagerNode();

private:
  void publish_stack_lifecycle();
  void handle_transition(
    const std::shared_ptr<custom_msgs_srvs::srv::SetStackLifecycleTransition::Request> request,
    std::shared_ptr<custom_msgs_srvs::srv::SetStackLifecycleTransition::Response> response);

  std::string robot_id() const;
  std::string resolve_lifecycle_node_fqn(const std::string & node_name) const;
  bool call_lifecycle_transition(const std::string & node_fqn, const std::string & transition, std::string * err);
  std::string query_lifecycle_state(const std::string & node_fqn);

  bool set_slam_mode(const std::string & mode, std::string * err);
  bool start_slam_child(const std::string & mode);
  void stop_slam_child();

  std::string slam_mode_;
  pid_t slam_pid_{-1};

  std::string mapper_params_file_;
  std::string localization_params_file_;
  std::string map_file_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string mapping_map_topic_;
  std::string localization_map_topic_;
  std::string slam_child_namespace_;
  bool use_sim_time_{true};

  std::vector<std::string> tracked_lifecycle_nodes_;
  // Separate group so timer/service callbacks can wait on lifecycle clients.
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Publisher<custom_msgs_srvs::msg::StackLifecycle>::SharedPtr stack_pub_;
  rclcpp::Service<custom_msgs_srvs::srv::SetStackLifecycleTransition>::SharedPtr transition_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace manager

#endif  // MANAGER__STACK_LIFECYCLE_MANAGER_NODE_HPP_
