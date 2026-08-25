#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <utility>

#include "custom_msgs_srvs/msg/localize_nav_command.hpp"
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

struct HistoricalMatch {
  MatchResult result;
  std::string map_name;
  std::string record_id;
};

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
    history_match_threshold_ =
      declare_parameter<double>("history_match_threshold", 0.55);
    auto_relocalize_on_startup_ =
      declare_parameter<bool>("auto_relocalize_on_startup", true);
    auto_retry_limit_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("auto_relocalize_retry_limit", 30)));
    const double auto_retry_period = std::max(
      0.1, declare_parameter<double>("auto_relocalize_retry_period_sec", 0.5));
    robot_id_ = trim_slashes(get_namespace());
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
    localize_command_pub_ =
      create_publisher<custom_msgs_srvs::msg::LocalizeNavCommand>("localize_nav_command", 10);

    record_service_ = create_service<custom_msgs_srvs::srv::RecordRelocalization>(
      "record_relocalization",
      std::bind(
        &RelocalizationNode::on_record, this, std::placeholders::_1, std::placeholders::_2));
    relocalize_service_ = create_service<custom_msgs_srvs::srv::Relocalize>(
      "relocalize",
      std::bind(
        &RelocalizationNode::on_relocalize, this, std::placeholders::_1,
        std::placeholders::_2));
    auto_timer_ = create_wall_timer(
      std::chrono::duration<double>(auto_retry_period),
      std::bind(&RelocalizationNode::on_auto_relocalize, this));

    RCLCPP_INFO(
      get_logger(),
      "ready: map_root=%s base=%s auto_startup=%s services=record_relocalization,relocalize",
      map_root_.c_str(), base_frame_.c_str(),
      auto_relocalize_on_startup_ ? "true" : "false");
  }

private:
  void on_status(const custom_msgs_srvs::msg::RobotStatus::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string next_map = message->current_map;
    const std::string next_status = message->robot_status;
    if (auto_relocalize_on_startup_ && !auto_relocalize_done_) {
      if (next_status == "localizing") {
        startup_localizing_seen_ = true;
      } else if (next_status == "localization_lost" &&
        (startup_localizing_seen_ || robot_status_.empty()) && !auto_relocalize_pending_)
      {
        auto_relocalize_pending_ = true;
        RCLCPP_INFO(
          get_logger(),
          "startup localization lost; scheduling current-map-first all-map relocalization");
      } else if (next_status == "shutdown") {
        auto_relocalize_pending_ = false;
        auto_relocalize_done_ = true;
      }
    }
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

  HistoricalMatch match_records_on_map(
    const std::string & map_name, const GridMap & map, const ScanData & scan)
  {
    HistoricalMatch best;
    best.map_name = map_name;
    const auto records = load_records(
      record_directory(map_name), map_name,
      map_root_ + "/" + map_name + "/" + map_name + "_points.json");
    for (const auto & record : records) {
      const MatchResult candidate = search_pose(map, scan, record.pose, &record, config_);
      if (candidate.valid && (!best.result.valid || candidate.score > best.result.score)) {
        best.result = candidate;
        best.record_id = record.id;
      }
    }
    return best;
  }

  bool find_history_match(
    const std::string & current_map, const GridMap & current_grid,
    const ScanData & scan, bool search_other_maps,
    HistoricalMatch * output, std::string * error)
  {
    HistoricalMatch current = match_records_on_map(current_map, current_grid, scan);
    HistoricalMatch best_seen = current;
    if (current.result.valid && current.result.score >= history_match_threshold_) {
      *output = std::move(current);
      return true;
    }
    if (!search_other_maps) {
      *error = current.result.valid ?
        "best history match below threshold on current map" :
        "no usable saved scan records on current map";
      return false;
    }

    HistoricalMatch best_other;
    const auto maps = discover_record_maps(map_root_, current_map);
    for (const auto & candidate_name : maps) {
      if (candidate_name == current_map) continue;
      GridMap candidate_grid;
      std::string map_error;
      const std::string yaml =
        map_root_ + "/" + candidate_name + "/" + candidate_name + ".yaml";
      if (!load_grid_map_from_yaml(yaml, &candidate_grid, &map_error)) {
        RCLCPP_WARN(
          get_logger(), "skip relocalization map %s: %s",
          candidate_name.c_str(), map_error.c_str());
        continue;
      }
      HistoricalMatch candidate =
        match_records_on_map(candidate_name, candidate_grid, scan);
      if (candidate.result.valid &&
        (!best_seen.result.valid || candidate.result.score > best_seen.result.score))
      {
        best_seen = candidate;
      }
      if (candidate.result.valid && candidate.result.score >= history_match_threshold_ &&
        (!best_other.result.valid || candidate.result.score > best_other.result.score))
      {
        best_other = std::move(candidate);
      }
    }
    if (best_other.result.valid) {
      *output = std::move(best_other);
      return true;
    }
    if (best_seen.result.valid) {
      *error = "best history match below threshold on map " + best_seen.map_name;
    } else {
      *error = "no usable saved scan records on any map";
    }
    return false;
  }

  bool publish_localization(
    const std::string & current_map, const HistoricalMatch & match,
    custom_msgs_srvs::srv::Relocalize::Response * response)
  {
    fill_pose_message(match.result.pose, &response->corrected_pose);
    if (match.map_name == current_map) {
      initial_pub_->publish(response->corrected_pose);
      response->message = "published corrected initial pose on current map " + current_map;
      return true;
    }
    if (robot_id_.empty()) {
      response->message = "cannot switch map from the root namespace";
      return false;
    }
    custom_msgs_srvs::msg::LocalizeNavCommand command;
    command.header.stamp = now();
    command.header.frame_id = map_frame_;
    command.robot_id = robot_id_;
    command.map_name = match.map_name;
    command.set_initial_pose = true;
    command.x = match.result.pose.x;
    command.y = match.result.pose.y;
    command.yaw = match.result.pose.yaw;
    localize_command_pub_->publish(command);
    response->message =
      "requested map switch and corrected initial pose on " + match.map_name;
    return true;
  }

  void on_auto_relocalize() {
    int attempt = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!auto_relocalize_pending_ || auto_relocalize_done_) return;
      attempt = ++auto_retry_attempts_;
    }
    auto request = std::make_shared<custom_msgs_srvs::srv::Relocalize::Request>();
    auto response = std::make_shared<custom_msgs_srvs::srv::Relocalize::Response>();
    request->mode = custom_msgs_srvs::srv::Relocalize::Request::MODE_HISTORY;
    on_relocalize(request, response);
    bool exhausted = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (response->success) {
        auto_relocalize_pending_ = false;
        auto_relocalize_done_ = true;
      } else if (attempt >= auto_retry_limit_) {
        auto_relocalize_pending_ = false;
        auto_relocalize_done_ = true;
        exhausted = true;
      }
    }
    if (response->success) {
      RCLCPP_INFO(
        get_logger(), "startup relocalization succeeded after %d attempt(s): %s",
        attempt, response->message.c_str());
    } else if (exhausted) {
      RCLCPP_ERROR(
        get_logger(), "startup relocalization stopped after %d attempt(s): %s",
        attempt, response->message.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "startup relocalization attempt %d/%d deferred: %s",
        attempt, auto_retry_limit_, response->message.c_str());
    }
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

    HistoricalMatch best;
    best.map_name = map_name;
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
      best.result = search_pose(map, scan, seed, nullptr, config_);
      if (best.result.valid && best.result.score >= pose_first_threshold_) {
        response->used_fallback = false;
      } else {
        response->used_fallback = true;
        best.result = MatchResult{};
      }
    }

    if (request->mode == custom_msgs_srvs::srv::Relocalize::Request::MODE_HISTORY ||
      response->used_fallback)
    {
      HistoricalMatch history;
      const bool search_other_maps =
        request->mode == custom_msgs_srvs::srv::Relocalize::Request::MODE_HISTORY;
      if (!find_history_match(map_name, map, scan, search_other_maps, &history, &error)) {
        response->message = error;
        return;
      }
      best = std::move(history);
      response->matched_record_id = best.record_id;
    }

    if (!best.result.valid) {
      response->message = "scan matching produced no candidate";
      return;
    }
    if (!publish_localization(map_name, best, response.get())) return;
    response->success = true;
    response->score = best.result.score;
    RCLCPP_INFO(
      get_logger(),
      "relocalized source_map=%s target_map=%s mode=%u fallback=%s "
      "score=%.3f map_score=%.3f scan_score=%.3f record=%s",
      map_name.c_str(), best.map_name.c_str(), request->mode,
      response->used_fallback ? "true" : "false",
      best.result.score, best.result.map_score, best.result.scan_score,
      response->matched_record_id.c_str());
  }

  std::mutex mutex_;
  std::string map_root_;
  std::string map_frame_;
  std::string base_frame_;
  std::string robot_id_;
  std::string current_map_;
  std::string grid_map_name_;
  std::string robot_status_;
  double pose_first_threshold_{0.55};
  double history_match_threshold_{0.55};
  double max_scan_age_sec_{1.0};
  MatchConfig config_;
  int auto_retry_limit_{30};
  int auto_retry_attempts_{0};
  bool auto_relocalize_on_startup_{true};
  bool startup_localizing_seen_{false};
  bool auto_relocalize_pending_{false};
  bool auto_relocalize_done_{false};
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
  rclcpp::Publisher<custom_msgs_srvs::msg::LocalizeNavCommand>::SharedPtr
    localize_command_pub_;
  rclcpp::Service<custom_msgs_srvs::srv::RecordRelocalization>::SharedPtr record_service_;
  rclcpp::Service<custom_msgs_srvs::srv::Relocalize>::SharedPtr relocalize_service_;
  rclcpp::TimerBase::SharedPtr auto_timer_;
};

}  // namespace relocalization

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<relocalization::RelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
