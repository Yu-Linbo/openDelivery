#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "custom_msgs_srvs/srv/query_semantic_location.hpp"
#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "manager/semantic_location_query.hpp"
#include "manager/semantic_raster_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace {

std::vector<std::string> split(const std::string & value) {
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    result.push_back(token);
  }
  return result;
}

std::vector<manager::SemanticRegion> parse_regions(
  const std::vector<std::string> & definitions, const rclcpp::Logger & logger) {
  std::vector<manager::SemanticRegion> result;
  for (const auto & definition : definitions) {
    const auto fields = split(definition);
    if (fields.size() != 5) {
      RCLCPP_WARN(logger, "skip semantic region %s: expected name,min_x,min_y,max_x,max_y", definition.c_str());
      continue;
    }
    try {
      manager::SemanticRegion region{
        fields[0], std::stod(fields[1]), std::stod(fields[2]),
        std::stod(fields[3]), std::stod(fields[4])};
      if (region.name.empty() || region.min_x > region.max_x || region.min_y > region.max_y) {
        throw std::invalid_argument("invalid bounds");
      }
      result.push_back(region);
    } catch (const std::exception &) {
      RCLCPP_WARN(logger, "skip invalid semantic region %s", definition.c_str());
    }
  }
  return result;
}

std::string format_distance(double distance) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << distance;
  return stream.str();
}

std::string trim(const std::string & value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

std::string join_path(const std::string & base, const std::string & child) {
  if (base.empty()) {
    return child;
  }
  return base.back() == '/' ? base + child : base + "/" + child;
}

bool valid_map_name(const std::string & map_name) {
  static const std::regex valid("^[A-Za-z0-9_-]+$");
  if (map_name.empty() || map_name.size() > 128 ||
    !std::regex_match(map_name, valid))
  {
    return false;
  }
  return map_name.size() < 8 ||
         map_name.substr(map_name.size() - 8) != "_mapping";
}

}  // namespace

namespace manager {

class SemanticLocationQueryNode : public rclcpp::Node {
public:
  SemanticLocationQueryNode()
  : rclcpp::Node("semantic_location_query") {
    declare_parameter<std::string>("robot_model", "OP1");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "");
    declare_parameter<std::string>("semantic_map_root", "");
    declare_parameter<double>("status_update_sec", 1.0);
    declare_parameter<bool>("enable_region_fallback", false);
    declare_parameter<std::vector<std::string>>("semantic_regions", std::vector<std::string>{});
    map_root_ = trim(get_parameter("semantic_map_root").as_string());
    map_frame_ = trim(get_parameter("map_frame").as_string());
    base_frame_ = trim(get_parameter("base_frame").as_string());
    if (base_frame_.empty()) {
      std::string robot_id = trim(get_namespace());
      while (!robot_id.empty() && robot_id.front() == '/') {
        robot_id.erase(robot_id.begin());
      }
      base_frame_ = robot_id.empty() ? "base_footprint" : robot_id + "/base_footprint";
    }
    enable_region_fallback_ = get_parameter("enable_region_fallback").as_bool();
    fallback_query_.set_regions(parse_regions(
      get_parameter("semantic_regions").as_string_array(), get_logger()));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    status_sub_ = create_subscription<custom_msgs_srvs::msg::RobotStatus>(
      "robot_status", rclcpp::QoS(10),
      std::bind(&SemanticLocationQueryNode::on_robot_status, this, std::placeholders::_1));
    heartbeat_client_ = create_client<custom_msgs_srvs::srv::SetHeartbeatParams>(
      "set_heartbeat_params");
    service_ = create_service<custom_msgs_srvs::srv::QuerySemanticLocation>(
      "query_semantic_location",
      std::bind(&SemanticLocationQueryNode::on_query, this, std::placeholders::_1, std::placeholders::_2));
    const double period = std::max(0.2, get_parameter("status_update_sec").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period), std::bind(&SemanticLocationQueryNode::update_status, this));
    RCLCPP_INFO(
      get_logger(), "semantic_location_query: model=%s tf=%s->%s map_root=%s fallback=%s",
      get_parameter("robot_model").as_string().c_str(), map_frame_.c_str(), base_frame_.c_str(),
      map_root_.empty() ? "<unset>" : map_root_.c_str(),
      enable_region_fallback_ ? "true" : "false");
  }

private:
  void on_robot_status(const custom_msgs_srvs::msg::RobotStatus::SharedPtr message) {
    const std::string map_name = trim(message->current_map);
    last_status_position_ = trim(message->current_position);
    if (map_name == requested_map_) {
      return;
    }
    // current_map is the rising-edge trigger. Repeated heartbeat messages for
    // the same map never reload the semantic YAML/image/legend.
    requested_map_ = map_name;
    loaded_map_.clear();
    if (!valid_map_name(map_name)) {
      RCLCPP_WARN(
        get_logger(), "semantic map unavailable for current_map=%s", map_name.c_str());
      return;
    }
    if (map_root_.empty()) {
      RCLCPP_ERROR(
        get_logger(), "semantic_map_root is empty; cannot load map=%s", map_name.c_str());
      return;
    }
    const std::string yaml = join_path(
      join_path(map_root_, map_name), map_name + "_semantic.yaml");
    std::string error;
    if (!loaded_map_.load(yaml, &error)) {
      RCLCPP_ERROR(get_logger(), "semantic map load failed: %s", error.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "semantic map loaded once: map=%s yaml=%s labels=%zu",
      map_name.c_str(), yaml.c_str(), loaded_map_.label_count());
  }


  bool pose(double * x, double * y) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
      *x = transform.transform.translation.x;
      *y = transform.transform.translation.y;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "semantic TF %s -> %s unavailable: %s",
        map_frame_.c_str(), base_frame_.c_str(), error.what());
      return false;
    }
    return std::isfinite(*x) && std::isfinite(*y);
  }

  std::string query_current(double x, double y) const {
    if (loaded_map_.valid()) {
      return loaded_map_.query_current_location(x, y);
    }
    return enable_region_fallback_ ? fallback_query_.query_current_location(x, y) : "unknown";
  }

  double query_distance(const std::string & name, double x, double y) const {
    if (loaded_map_.valid()) {
      return loaded_map_.distance_to(name, x, y);
    }
    return enable_region_fallback_ ? fallback_query_.distance_to(name, x, y) : -1.0;
  }

  std::vector<SemanticDistance> query_nearest(
    double x, double y, const std::string & exclude) const
  {
    if (loaded_map_.valid()) {
      return loaded_map_.nearest(x, y, 3, exclude);
    }
    return enable_region_fallback_ ? fallback_query_.nearest(x, y, 3, exclude) :
           std::vector<SemanticDistance>{};
  }

  std::string current_position(double x, double y) const {
    const std::string current = query_current(x, y);
    std::string summary = current + ";";
    for (const auto & item : query_nearest(x, y, current == "unknown" ? "" : current)) {
      summary += item.name + ":" + format_distance(item.distance) + ";";
    }
    return summary;
  }

  void update_status() {
    double x = 0.0;
    double y = 0.0;
    const std::string summary = pose(&x, &y) ? current_position(x, y) : "unknown;";
    if (summary == last_status_position_ || update_in_flight_ ||
      !heartbeat_client_->wait_for_service(std::chrono::milliseconds(50)))
    {
      return;
    }
    auto request = std::make_shared<custom_msgs_srvs::srv::SetHeartbeatParams::Request>();
    request->current_position = summary;
    request->task_progress = -1.0;
    update_in_flight_ = true;
    heartbeat_client_->async_send_request(
      request,
      [this, summary](
        rclcpp::Client<custom_msgs_srvs::srv::SetHeartbeatParams>::SharedFuture future)
      {
        update_in_flight_ = false;
        try {
          const auto response = future.get();
          if (response && response->success) {
            last_status_position_ = summary;
          } else {
            RCLCPP_WARN(get_logger(), "set current_position was rejected");
          }
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "set current_position failed: %s", error.what());
        }
      });
  }

  void on_query(
    const std::shared_ptr<custom_msgs_srvs::srv::QuerySemanticLocation::Request> request,
    std::shared_ptr<custom_msgs_srvs::srv::QuerySemanticLocation::Response> response) {
    double x = 0.0;
    double y = 0.0;
    if (!pose(&x, &y)) {
      response->success = false;
      response->current_position = "unknown";
      response->distance = -1.0;
      response->message = "semantic TF unavailable";
      return;
    }
    response->current_position = query_current(x, y);
    response->distance = query_distance(request->semantic_name, x, y);
    response->success = response->distance >= 0.0;
    response->message = response->success ? "ok" :
      (loaded_map_.valid() ? "unknown semantic location" : "semantic map unavailable");
  }

  SemanticLocationQuery fallback_query_;
  SemanticRasterMap loaded_map_;
  bool enable_region_fallback_{false};
  bool update_in_flight_{false};
  std::string map_root_;
  std::string map_frame_;
  std::string base_frame_;
  std::string requested_map_;
  std::string last_status_position_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<custom_msgs_srvs::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Client<custom_msgs_srvs::srv::SetHeartbeatParams>::SharedPtr heartbeat_client_;
  rclcpp::Service<custom_msgs_srvs::srv::QuerySemanticLocation>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace manager

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manager::SemanticLocationQueryNode>());
  rclcpp::shutdown();
  return 0;
}
