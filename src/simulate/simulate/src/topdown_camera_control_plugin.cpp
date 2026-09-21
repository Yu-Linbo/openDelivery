#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <mutex>
#include <string>
#include <utility>

namespace gazebo {

class TopdownCameraControlPlugin : public WorldPlugin {
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = std::move(world);
    model_name_ = sdf->HasElement("model_name")
      ? sdf->Get<std::string>("model_name")
      : "topdown_camera";
    topic_name_ = sdf->HasElement("topic_name")
      ? sdf->Get<std::string>("topic_name")
      : "/open_delivery/topdown_camera/pose";

    node_ = gazebo_ros::Node::Get(sdf);
    subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_name_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        std::lock_guard<std::mutex> guard(pose_mutex_);
        pending_pose_ = message->pose;
        pose_pending_ = true;
      });
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TopdownCameraControlPlugin::OnUpdate, this));

    RCLCPP_INFO(
      node_->get_logger(),
      "topdown camera control ready: model=%s topic=%s",
      model_name_.c_str(), topic_name_.c_str());
  }

private:
  void OnUpdate() {
    geometry_msgs::msg::Pose pose;
    {
      std::lock_guard<std::mutex> guard(pose_mutex_);
      if (!pose_pending_) {
        return;
      }
      pose = pending_pose_;
      pose_pending_ = false;
    }

    const auto model = world_->ModelByName(model_name_);
    if (!model) {
      return;
    }
    const ignition::math::Pose3d world_pose(
      pose.position.x,
      pose.position.y,
      pose.position.z,
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
    model->SetWorldPose(world_pose, true, true);
  }

  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  event::ConnectionPtr update_connection_;
  std::string model_name_;
  std::string topic_name_;
  std::mutex pose_mutex_;
  geometry_msgs::msg::Pose pending_pose_;
  bool pose_pending_{false};
};

GZ_REGISTER_WORLD_PLUGIN(TopdownCameraControlPlugin)

}  // namespace gazebo
