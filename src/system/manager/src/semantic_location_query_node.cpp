#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "custom_msgs_srvs/srv/query_semantic_location.hpp"
#include "custom_msgs_srvs/srv/set_heartbeat_params.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "manager/semantic_location_query.hpp"
#include "rclcpp/rclcpp.hpp"

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

}  // namespace

namespace manager {

class SemanticLocationQueryNode : public rclcpp::Node {
public:
  SemanticLocationQueryNode()
  : rclcpp::Node("semantic_location_query") {
    declare_parameter<std::string>("robot_model", "OP1");
    declare_parameter<std::string>("localization_pose_topic", "amcl_pose");
    declare_parameter<double>("status_update_sec", 1.0);
    declare_parameter<double>("position_covariance_max", 0.45);
    declare_parameter<double>("pose_timeout_sec", 2.5);
    declare_parameter<std::vector<std::string>>("semantic_regions", std::vector<std::string>{});
    query_ = SemanticLocationQuery(parse_regions(
      get_parameter("semantic_regions").as_string_array(), get_logger()));
    const auto pose_topic = get_parameter("localization_pose_topic").as_string();
    covariance_max_ = get_parameter("position_covariance_max").as_double();
    pose_timeout_sec_ = get_parameter("pose_timeout_sec").as_double();
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic, rclcpp::QoS(10),
      std::bind(&SemanticLocationQueryNode::on_pose, this, std::placeholders::_1));
    heartbeat_client_ = create_client<custom_msgs_srvs::srv::SetHeartbeatParams>(
      "set_heartbeat_params");
    service_ = create_service<custom_msgs_srvs::srv::QuerySemanticLocation>(
      "query_semantic_location",
      std::bind(&SemanticLocationQueryNode::on_query, this, std::placeholders::_1, std::placeholders::_2));
    const double period = std::max(0.2, get_parameter("status_update_sec").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period), std::bind(&SemanticLocationQueryNode::update_status, this));
    RCLCPP_INFO(
      get_logger(), "semantic_location_query: model=%s regions=%zu pose_topic=%s",
      get_parameter("robot_model").as_string().c_str(), query_.nearest(0.0, 0.0, 999).size(), pose_topic.c_str());
  }

private:
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & covariance = message->pose.covariance;
    if (!std::isfinite(covariance[0]) || !std::isfinite(covariance[7]) ||
      covariance[0] > covariance_max_ || covariance[7] > covariance_max_)
    {
      have_pose_ = false;
      return;
    }
    x_ = message->pose.pose.position.x;
    y_ = message->pose.pose.position.y;
    have_pose_ = std::isfinite(x_) && std::isfinite(y_);
    last_pose_received_ = std::chrono::steady_clock::now();
  }

  bool pose(double * x, double * y) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_pose_received_).count();
    if (!have_pose_ || age > pose_timeout_sec_) {
      return false;
    }
    *x = x_;
    *y = y_;
    return true;
  }

  std::string current_position(double x, double y) const {
    const std::string current = query_.query_current_location(x, y);
    std::string summary = current + ";";
    for (const auto & item : query_.nearest(x, y, 3, current == "unknown" ? "" : current)) {
      summary += item.name + ":" + format_distance(item.distance) + ";";
    }
    return summary;
  }

  void update_status() {
    double x = 0.0;
    double y = 0.0;
    const std::string summary = pose(&x, &y) ? current_position(x, y) : "unknown;";
    if (!heartbeat_client_->wait_for_service(std::chrono::milliseconds(50))) {
      return;
    }
    auto request = std::make_shared<custom_msgs_srvs::srv::SetHeartbeatParams::Request>();
    request->current_position = summary;
    request->task_progress = -1.0;
    heartbeat_client_->async_send_request(request);
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
      response->message = "localization pose unavailable";
      return;
    }
    response->current_position = query_.query_current_location(x, y);
    response->distance = query_.distance_to(request->semantic_name, x, y);
    response->success = response->distance >= 0.0;
    response->message = response->success ? "ok" : "unknown semantic location";
  }

  SemanticLocationQuery query_;
  mutable std::mutex mutex_;
  bool have_pose_{false};
  double x_{0.0};
  double y_{0.0};
  double covariance_max_{0.45};
  double pose_timeout_sec_{2.5};
  std::chrono::steady_clock::time_point last_pose_received_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
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
