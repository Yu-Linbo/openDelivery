#include <algorithm>
#include <chrono>
#include <cctype>
#include <memory>
#include <mutex>
#include <string>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace chassis_state_machine {

using namespace std::chrono_literals;

class ChassisStateMachineNode : public rclcpp::Node {
public:
  ChassisStateMachineNode() : Node("chassis_state_machine") {
    command_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("command_timeout_sec", 0.5));

    advance_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "advance/cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        advance_cmd_ = *msg;
        advance_stamp_ = now();
        advance_received_ = true;
      });
    navigation_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "navagtion/cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        navigation_cmd_ = *msg;
        navigation_stamp_ = now();
        navigation_received_ = true;
      });
    status_sub_ = create_subscription<custom_msgs_srvs::msg::RobotStatus>(
      "robot_status", rclcpp::QoS(10),
      [this](custom_msgs_srvs::msg::RobotStatus::SharedPtr msg) {
        std::string mode = msg->control_status;
        std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
          return static_cast<char>(std::toupper(c));
        });
        if (mode != "AUTO" && mode != "JOY" && mode != "MANUAL") {
          RCLCPP_WARN(get_logger(), "ignoring invalid control_status '%s'", mode.c_str());
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_status_ != mode) {
          RCLCPP_INFO(get_logger(), "control_status: %s -> %s",
            control_status_.c_str(), mode.c_str());
          control_status_ = mode;
        }
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    timer_ = create_wall_timer(100ms, std::bind(&ChassisStateMachineNode::publish_command, this));
    RCLCPP_INFO(get_logger(), "ready: AUTO=navigation, JOY=advance, MANUAL=zero; output 10 Hz");
  }

private:
  bool fresh(bool received, const rclcpp::Time & stamp, const rclcpp::Time & current) const {
    return received && (current - stamp) <= command_timeout_;
  }

  void publish_command() {
    geometry_msgs::msg::Twist output;
    const auto current = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (control_status_ == "JOY" && fresh(advance_received_, advance_stamp_, current)) {
        output = advance_cmd_;
      } else if (
        control_status_ == "AUTO" && fresh(navigation_received_, navigation_stamp_, current)) {
        output = navigation_cmd_;
      }
      // MANUAL intentionally publishes zero in simulation. A real chassis driver can use
      // control_status to release motor holding torque before this output stage.
    }
    cmd_pub_->publish(output);
  }

  std::mutex mutex_;
  std::string control_status_{"AUTO"};
  geometry_msgs::msg::Twist advance_cmd_;
  geometry_msgs::msg::Twist navigation_cmd_;
  rclcpp::Time advance_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time navigation_stamp_{0, 0, RCL_ROS_TIME};
  bool advance_received_{false};
  bool navigation_received_{false};
  rclcpp::Duration command_timeout_{0, 0};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr advance_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_sub_;
  rclcpp::Subscription<custom_msgs_srvs::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace chassis_state_machine

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<chassis_state_machine::ChassisStateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
