#ifndef MANAGER__STACK_LIFECYCLE_MANAGER_NODE_HPP_
#define MANAGER__STACK_LIFECYCLE_MANAGER_NODE_HPP_

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "custom_msgs_srvs/msg/stack_lifecycle.hpp"
#include "custom_msgs_srvs/srv/set_stack_lifecycle_transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace manager {

class StackLifecycleManagerNode : public rclcpp::Node {
public:
  StackLifecycleManagerNode();
  ~StackLifecycleManagerNode() override;

private:
  void publish_stack_lifecycle();
  void start_initial_modules();
  void handle_transition(
    const std::shared_ptr<custom_msgs_srvs::srv::SetStackLifecycleTransition::Request> request,
    std::shared_ptr<custom_msgs_srvs::srv::SetStackLifecycleTransition::Response> response);

  std::string robot_id() const;
  std::string slam_node_fqn(const std::string & leaf) const;
  std::string resolve_lifecycle_node_fqn(const std::string & node_name) const;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client(
    const std::string & node_fqn);
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client(
    const std::string & node_fqn);
  bool call_lifecycle_transition(const std::string & node_fqn, const std::string & transition, std::string * err);
  std::string query_lifecycle_state(const std::string & node_fqn);
  bool navigation_active();

  bool set_slam_mode(const std::string & mode, std::string * err);
  bool start_slam_child(const std::string & mode);
  void stop_slam_child();

  /** Load occupancy yaml via nav2 map_server onto static_map_topic_ (default /<robot>/map). */
  bool ensure_static_map_loaded(std::string * err = nullptr);
  bool start_map_server(std::string * err = nullptr);
  void stop_map_server();

  std::string slam_mode_;
  pid_t slam_pid_{-1};
  pid_t map_server_pid_{-1};
  std::string map_server_yaml_;

  std::string mapper_params_file_;
  std::string localization_params_file_;
  std::string map_file_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string mapping_map_topic_;
  std::string static_map_topic_;
  std::string slam_child_namespace_;
  bool use_sim_time_{true};

  std::vector<std::string> tracked_lifecycle_nodes_;
  // Separate group so timer/service callbacks can wait on lifecycle clients.
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>
    get_state_clients_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr>
    change_state_clients_;
  std::mutex lifecycle_clients_mutex_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr navigation_active_client_;
  rclcpp::Publisher<custom_msgs_srvs::msg::StackLifecycle>::SharedPtr stack_pub_;
  rclcpp::Service<custom_msgs_srvs::srv::SetStackLifecycleTransition>::SharedPtr transition_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr initial_start_timer_;
  std::string initial_slam_mode_;
  std::string robot_id_;
  std::atomic<bool> initialization_complete_{false};
};

}  // namespace manager

#endif  // MANAGER__STACK_LIFECYCLE_MANAGER_NODE_HPP_
