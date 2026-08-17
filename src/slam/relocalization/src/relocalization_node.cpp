#include <cerrno>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <utility>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "custom_msgs_srvs/srv/record_relocalization.hpp"
#include "custom_msgs_srvs/srv/relocalize.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "relocalization/relocalization_core.hpp"

namespace relocalization {
namespace {

std::string trim_slashes(std::string value) {
  while (!value.empty() && value.front() == '/') value.erase(value.begin());
  while (!value.empty() && value.back() == '/') value.pop_back();
  return value;
}

bool ensure_directory(const std::string & path, std::string * error) {
  if (::mkdir(path.c_str(), 0755) == 0 || errno == EEXIST) return true;
  if (error) *error = std::string("cannot create ") + path + ": " + std::strerror(errno);
  return false;
}

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

class RelocalizationNode : public rclcpp::Node {
public:
  RelocalizationNode()
  : Node("relocalization"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    map_root_ = declare_parameter<std::string>("map_root", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "");
    if (base_frame_.empty()) {
      const std::string ns = trim_slashes(get_namespace());
      base_frame_ = ns.empty() ? "base_footprint" : ns + "/base_footprint";
    }
    pose_first_threshold_ = declare_parameter<double>("pose_first_threshold", 0.55);
    max_scan_age_sec_ = declare_parameter<double>("max_scan_age_sec", 1.0);
    config_.search_xy = declare_parameter<double>("search_xy", config_.search_xy);
    config_.search_yaw = declare_parameter<double>("search_yaw", config_.search_yaw);
    config_.map_weight = declare_parameter<double>("map_weight", config_.map_weight);

    status_sub_ = create_subscription<custom_msgs_srvs::msg::RobotStatus>(
      "robot_status", rclcpp::QoS(10),
      std::bind(&RelocalizationNode::on_status, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_2d", rclcpp::SensorDataQoS(),
      std::bind(&RelocalizationNode::on_scan, this, std::placeholders::_1));
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", map_qos,
      std::bind(&RelocalizationNode::on_map, this, std::placeholders::_1));
    initial_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial", rclcpp::QoS(10));

    record_service_ = create_service<custom_msgs_srvs::srv::RecordRelocalization>(
      "record_relocalization",
      std::bind(
        &RelocalizationNode::on_record, this, std::placeholders::_1, std::placeholders::_2));
    relocalize_service_ = create_service<custom_msgs_srvs::srv::Relocalize>(
      "relocalize",
      std::bind(
        &RelocalizationNode::on_relocalize, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "ready: map_root=%s base=%s services=record_relocalization,relocalize",
      map_root_.c_str(), base_frame_.c_str());
  }

private:
  void on_status(const custom_msgs_srvs::msg::RobotStatus::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string next_map = message->current_map;
    if (!current_map_.empty() && !next_map.empty() && next_map != current_map_) {
      map_ready_ = false;
      RCLCPP_INFO(
        get_logger(), "map changed %s -> %s; waiting for new OccupancyGrid",
        current_map_.c_str(), next_map.c_str());
    }
    current_map_ = next_map;
    robot_status_ = message->robot_status;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_scan_ = *message;
    scan_received_at_ = now();
    has_scan_ = true;
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
    GridMap grid;
    grid.width = message->info.width;
    grid.height = message->info.height;
    grid.resolution = message->info.resolution;
    grid.origin.x = message->info.origin.position.x;
    grid.origin.y = message->info.origin.position.y;
    grid.origin.yaw = tf2::getYaw(message->info.origin.orientation);
    grid.cells = message->data;
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map_ = std::move(grid);
    grid_map_name_ = current_map_;
    map_ready_ = grid_map_.valid();
  }

  bool snapshot_inputs(
    std::string * map_name, std::string * status, sensor_msgs::msg::LaserScan * scan,
    GridMap * map, std::string * error)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_map_.empty()) {
      *error = "RobotStatus.current_map is empty";
      return false;
    }
    if (!valid_record_id(current_map_)) {
      *error = "RobotStatus.current_map is invalid";
      return false;
    }
    if (!has_scan_ || (now() - scan_received_at_).seconds() > max_scan_age_sec_) {
      *error = "no fresh scan_2d frame";
      return false;
    }
    if (!map_ready_ || !grid_map_.valid() ||
      (!grid_map_name_.empty() && grid_map_name_ != current_map_))
    {
      *error = "current map OccupancyGrid is not ready";
      return false;
    }
    *map_name = current_map_;
    *status = robot_status_;
    *scan = last_scan_;
    *map = grid_map_;
    return true;
  }

  bool scan_data_from_message(
    const sensor_msgs::msg::LaserScan & message, ScanData * output, std::string * error)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, message.header.frame_id, tf2::TimePointZero);
      output->laser_in_base.x = transform.transform.translation.x;
      output->laser_in_base.y = transform.transform.translation.y;
      output->laser_in_base.yaw = tf2::getYaw(transform.transform.rotation);
    } catch (const std::exception & exception) {
      *error = std::string("cannot transform laser into base frame: ") + exception.what();
      return false;
    }
    output->angle_min = message.angle_min;
    output->angle_increment = message.angle_increment;
    output->range_min = message.range_min;
    output->range_max = message.range_max;
    output->ranges = message.ranges;
    return !output->ranges.empty();
  }

  bool current_pose(Pose2D * pose, std::string * error) {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
      pose->x = transform.transform.translation.x;
      pose->y = transform.transform.translation.y;
      pose->yaw = tf2::getYaw(transform.transform.rotation);
      return true;
    } catch (const std::exception & exception) {
      *error = std::string("cannot transform base into map frame: ") + exception.what();
      return false;
    }
  }

  std::string record_directory(const std::string & map_name) const {
    return map_root_ + "/" + map_name + "/relocalization";
  }

  void on_record(
    const std::shared_ptr<custom_msgs_srvs::srv::RecordRelocalization::Request> request,
    std::shared_ptr<custom_msgs_srvs::srv::RecordRelocalization::Response> response)
  {
    if (!valid_record_id(request->record_id)) {
      response->message = "record_id must match [A-Za-z0-9][A-Za-z0-9_-]{0,63}";
      return;
    }
    std::string map_name, status, error;
    sensor_msgs::msg::LaserScan scan_message;
    GridMap ignored_map;
    if (!snapshot_inputs(&map_name, &status, &scan_message, &ignored_map, &error)) {
      response->message = error;
      return;
    }
    if (status != "ready") {
      response->message = "recording requires robot_status=ready";
      return;
    }
    ScanRecord record;
    record.id = request->record_id;
    record.map_name = map_name;
    if (!current_pose(&record.pose, &error) ||
      !scan_data_from_message(scan_message, &record.scan, &error))
    {
      response->message = error;
      return;
    }
    if (map_root_.empty()) {
      response->message = "map_root parameter is empty";
      return;
    }
    const std::string map_folder = map_root_ + "/" + map_name;
    const std::string directory = record_directory(map_name);
    if (!ensure_directory(map_folder, &error) || !ensure_directory(directory, &error)) {
      response->message = error;
      return;
    }
    const std::string path = directory + "/" + record.id + ".rloc";
    if (!save_record(path, record, &error)) {
      response->message = error;
      return;
    }
    response->success = true;
    response->message = "recorded";
    response->map_name = map_name;
    response->pose.position.x = record.pose.x;
    response->pose.position.y = record.pose.y;
    response->pose.orientation = yaw_quaternion(record.pose.yaw);
    response->storage_path = path;
    RCLCPP_INFO(
      get_logger(), "saved relocalization record map=%s id=%s beams=%zu",
      map_name.c_str(), record.id.c_str(), record.scan.ranges.size());
  }

  void fill_pose_message(
    const Pose2D & pose, geometry_msgs::msg::PoseWithCovarianceStamped * message)
  {
    message->header.stamp = now();
    message->header.frame_id = map_frame_;
    message->pose.pose.position.x = pose.x;
    message->pose.pose.position.y = pose.y;
    message->pose.pose.orientation = yaw_quaternion(pose.yaw);
    message->pose.covariance.fill(0.0);
    message->pose.covariance[0] = 0.04;
    message->pose.covariance[7] = 0.04;
    message->pose.covariance[35] = 0.03;
  }

  void on_relocalize(
    const std::shared_ptr<custom_msgs_srvs::srv::Relocalize::Request> request,
    std::shared_ptr<custom_msgs_srvs::srv::Relocalize::Response> response)
  {
    if (request->mode > custom_msgs_srvs::srv::Relocalize::Request::MODE_POSE_FIRST) {
      response->message = "mode must be 0 (history) or 1 (pose first)";
      return;
    }
    std::string map_name, status, error;
    sensor_msgs::msg::LaserScan scan_message;
    GridMap map;
    if (!snapshot_inputs(&map_name, &status, &scan_message, &map, &error)) {
      response->message = error;
      return;
    }
    if (status == "shutdown") {
      response->message = "relocalization unavailable while robot_status=shutdown";
      return;
    }
    ScanData scan;
    if (!scan_data_from_message(scan_message, &scan, &error)) {
      response->message = error;
      return;
    }

    MatchResult best;
    if (request->mode == custom_msgs_srvs::srv::Relocalize::Request::MODE_POSE_FIRST) {
      const auto & input = request->pose;
      if (!input.header.frame_id.empty() && input.header.frame_id != map_frame_) {
        response->message = "input pose must use map frame";
        return;
      }
      Pose2D seed{
        input.pose.pose.position.x,
        input.pose.pose.position.y,
        tf2::getYaw(input.pose.pose.orientation)};
      best = search_pose(map, scan, seed, nullptr, config_);
      if (best.valid && best.score >= pose_first_threshold_) {
        response->used_fallback = false;
      } else {
        response->used_fallback = true;
        best = MatchResult{};
      }
    }

    if (request->mode == custom_msgs_srvs::srv::Relocalize::Request::MODE_HISTORY ||
      response->used_fallback)
    {
      const auto records = load_records(
        record_directory(map_name), map_name,
        map_root_ + "/" + map_name + "/" + map_name + "_points.json");
      if (records.empty()) {
        response->message = "no saved scan records for current map";
        return;
      }
      for (const auto & record : records) {
        const MatchResult candidate = search_pose(map, scan, record.pose, &record, config_);
        if (candidate.valid && (!best.valid || candidate.score > best.score)) {
          best = candidate;
          response->matched_record_id = record.id;
        }
      }
    }

    if (!best.valid) {
      response->message = "scan matching produced no candidate";
      return;
    }
    fill_pose_message(best.pose, &response->corrected_pose);
    initial_pub_->publish(response->corrected_pose);
    response->success = true;
    response->score = best.score;
    response->message = "published corrected initial pose";
    RCLCPP_INFO(
      get_logger(),
      "relocalized map=%s mode=%u fallback=%s score=%.3f map=%.3f scan=%.3f record=%s",
      map_name.c_str(), request->mode, response->used_fallback ? "true" : "false",
      best.score, best.map_score, best.scan_score,
      response->matched_record_id.c_str());
  }

  std::mutex mutex_;
  std::string map_root_;
  std::string map_frame_;
  std::string base_frame_;
  std::string current_map_;
  std::string grid_map_name_;
  std::string robot_status_;
  double pose_first_threshold_{0.55};
  double max_scan_age_sec_{1.0};
  MatchConfig config_;
  bool has_scan_{false};
  bool map_ready_{false};
  rclcpp::Time scan_received_at_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::LaserScan last_scan_;
  GridMap grid_map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<custom_msgs_srvs::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pub_;
  rclcpp::Service<custom_msgs_srvs::srv::RecordRelocalization>::SharedPtr record_service_;
  rclcpp::Service<custom_msgs_srvs::srv::Relocalize>::SharedPtr relocalize_service_;
};

}  // namespace relocalization

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<relocalization::RelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
