#pragma once

#include <algorithm>
#include <string>
#include <vector>

namespace log_bag {

// Keep the bag focused on offline replay and task diagnosis. Explicit topics
// also let rosbag wait for late publishers without recording every Nav2 debug,
// action, lifecycle, costmap, Gazebo, or rosout topic in the namespace.
inline std::vector<std::string> recording_topics(const std::string & robot_name) {
  std::vector<std::string> topics = {"/clock", "/tf", "/tf_static"};
  const std::string prefix = "/" + robot_name;
  for (const char * suffix : {
      "/amcl_pose",
      "/cmd_vel",
      "/fake_elevator/command",
      "/fake_elevator/info",
      "/fake_elevator/status",
      "/front_camera/image_raw",
      "/front_down_camera/image_raw",
      "/imu/data",
      "/localize_nav_command",
      "/navigation/goal_pose",
      "/navigation/local_plan",
      "/navigation/odom",
      "/navigation/plan",
      "/navigation/received_global_plan",
      "/navigation/task_command",
      "/navigation/task_info",
      "/navigation/task_status",
      "/navigation/transformed_global_plan",
      "/odom",
      "/robot_status",
      "/scan_2d",
      "/task_command",
      "/task_info",
      "/task_status"})
  {
    topics.push_back(prefix + suffix);
  }
  std::sort(topics.begin(), topics.end());
  topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
  return topics;
}

}  // namespace log_bag
