#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace manager {

class TfLocalizationPoseNode : public rclcpp::Node {
public:
  TfLocalizationPoseNode()
  : Node("tf_localization_pose"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "amcl_pose");
    covariance_xy_ = std::max(0.0, declare_parameter<double>("covariance_xy", 0.04));
    covariance_yaw_ = std::max(0.0, declare_parameter<double>("covariance_yaw", 0.03));
    publish_period_sec_ = std::max(
      0.02, declare_parameter<double>("publish_period_sec", 0.10));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(10));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_sec_),
      std::bind(&TfLocalizationPoseNode::publish_pose, this));

    RCLCPP_INFO(
      get_logger(), "TF localization pose bridge: %s -> %s topic=%s",
      map_frame_.c_str(), base_frame_.c_str(), pose_topic_.c_str());
  }

private:
  void publish_pose() {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "waiting for localization TF %s -> %s: %s",
        map_frame_.c_str(), base_frame_.c_str(), exception.what());
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = transform.transform.translation.x;
    pose.pose.pose.position.y = transform.transform.translation.y;
    pose.pose.pose.position.z = transform.transform.translation.z;
    pose.pose.pose.orientation = transform.transform.rotation;
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0] = covariance_xy_;
    pose.pose.covariance[7] = covariance_xy_;
    pose.pose.covariance[35] = covariance_yaw_;
    pose_pub_->publish(pose);
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string pose_topic_;
  double covariance_xy_{0.04};
  double covariance_yaw_{0.03};
  double publish_period_sec_{0.10};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace manager

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manager::TfLocalizationPoseNode>());
  rclcpp::shutdown();
  return 0;
}
