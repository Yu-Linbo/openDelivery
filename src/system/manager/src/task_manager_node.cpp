#include "manager/task_manager_node.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <chrono>
#include <cctype>
#include <functional>
#include <future>
#include <fstream>
#include <json/json.h>
#include <sstream>
#include <thread>
#include <unordered_set>

#include "custom_msgs_srvs/msg/robot_status.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

namespace {

std::string strip(const std::string & s) {
  const char * ws = " \t\n\r";
  const auto a = s.find_first_not_of(ws);
  if (a == std::string::npos) {
    return {};
  }
  const auto b = s.find_last_not_of(ws);
  return s.substr(a, b - a + 1);
}

std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

std::string leaf_namespace_id(const std::string & raw_ns) {
  std::string ns = raw_ns;
  if (!ns.empty() && ns.front() == '/') {
    ns.erase(0, 1);
  }
  while (!ns.empty() && ns.back() == '/') {
    ns.pop_back();
  }
  if (ns.empty()) {
    return {};
  }
  const auto p = ns.find_last_of('/');
  if (p == std::string::npos) {
    return ns;
  }
  return ns.substr(p + 1);
}

bool valid_robot_status(const std::string & s) {
  static const std::unordered_set<std::string> k{
    "initializing", "localizing", "localization_lost", "ready", "shutdown"};
  return k.count(s) > 0;
}

bool is_elevator_position(const std::string & summary) {
  const std::string::size_type separator = summary.find(';');
  const std::string current = to_lower(strip(summary.substr(0, separator)));
  return current == "elevator" || current == "elevator_inside" ||
         current == "lift" || current == "电梯" ||
         current == "电梯内" || current == "电梯区域" ||
         current == "电梯轿厢";
}

bool valid_task_status(const std::string & s) {
  static const std::unordered_set<std::string> k{
    "idle", "mapping", "delivery", "cleaning", "patrolling"};
  return k.count(s) > 0;
}

bool terminal_task_status(const std::string & s) {
  return s == "Finished" || s == "Failed" || s == "Terminated";
}

bool valid_map_id(const std::string & value) {
  if (value.empty() || !std::isalnum(static_cast<unsigned char>(value.front()))) return false;
  return std::all_of(value.begin(), value.end(), [](unsigned char c) {
    return std::isalnum(c) || c == '_' || c == '-';
  });
}

geometry_msgs::msg::Pose pose_from_point(const Json::Value & row) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = row["x"].asDouble();
  pose.position.y = row["y"].asDouble();
  const double yaw = row.get("yaw", 0.0).asDouble();
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

struct ElevatorPoints {
  geometry_msgs::msg::Pose inside;
  geometry_msgs::msg::Pose waiting;
  bool has_inside{false};
  bool has_waiting{false};
};

bool load_elevator_points(
  const std::string & map_root, const std::string & floor,
  ElevatorPoints * output, std::string * error)
{
  if (map_root.empty()) {
    *error = "task_manager map_root is empty";
    return false;
  }
  if (!valid_map_id(floor)) {
    *error = "invalid floor id: " + floor;
    return false;
  }
  const std::string path = map_root + "/" + floor + "/" + floor + "_points.json";
  std::ifstream stream(path);
  if (!stream) {
    *error = "elevator points file missing: " + path;
    return false;
  }
  Json::Value document;
  Json::CharReaderBuilder builder;
  std::string parse_error;
  if (!Json::parseFromStream(builder, stream, &document, &parse_error)) {
    *error = "invalid elevator points json " + path + ": " + parse_error;
    return false;
  }
  const Json::Value & rows = document.isArray() ? document : document["points"];
  if (!rows.isArray()) {
    *error = "elevator points json has no points array: " + path;
    return false;
  }
  size_t inside_count = 0;
  size_t waiting_count = 0;
  for (const auto & row : rows) {
    if (!row.isObject() || !row["x"].isNumeric() || !row["y"].isNumeric()) continue;
    const std::string type = to_lower(strip(row.get("type", "").asString()));
    if (type == "elevator" || type == "elevator_inside") {
      output->inside = pose_from_point(row);
      output->has_inside = true;
      ++inside_count;
    } else if (type == "elevator_waiting") {
      output->waiting = pose_from_point(row);
      output->has_waiting = true;
      ++waiting_count;
    }
  }
  if (inside_count != 1 || waiting_count != 1) {
    std::ostringstream detail;
    detail << floor << " requires exactly one elevator_inside and one elevator_waiting point"
           << " (inside=" << inside_count << ", waiting=" << waiting_count << ")";
    *error = detail.str();
    return false;
  }
  return true;
}

geometry_msgs::msg::Pose reverse_pose(geometry_msgs::msg::Pose pose) {
  constexpr double kPi = 3.14159265358979323846;
  const double yaw = 2.0 * std::atan2(pose.orientation.z, pose.orientation.w) + kPi;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

template<typename FutureT>
bool wait_client_future_no_spin(const FutureT & fut, std::chrono::seconds timeout) {
  const auto step = std::chrono::milliseconds(20);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (fut.wait_for(step) == std::future_status::ready) {
      return true;
    }
    std::this_thread::sleep_for(step);
  }
  return fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

}  // namespace

namespace manager {

using RobotStatusMsg = custom_msgs_srvs::msg::RobotStatus;
using Srv = custom_msgs_srvs::srv::SetHeartbeatParams;

TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("task_manager", options) {
  declare_parameter<std::string>("initial_pose_topic", "initial");
  initial_pose_topic_ = strip(get_parameter("initial_pose_topic").as_string());
  if (initial_pose_topic_.empty()) {
    initial_pose_topic_ = "initial";
  }
  declare_parameter<std::string>("map_root", "");
  map_root_ = strip(get_parameter("map_root").as_string());

  // Client responses must not share the MutuallyExclusive group used by
  // localize_nav / set_robot_task callbacks (otherwise wait times out ~8s).
  hb_client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  hb_client_ = create_client<custom_msgs_srvs::srv::SetHeartbeatParams>(
    "set_heartbeat_params",
    rmw_qos_profile_services_default,
    hb_client_cb_group_);
  srv_ = create_service<custom_msgs_srvs::srv::SetRobotTask>(
    "set_robot_task",
    std::bind(&TaskManagerNode::on_set_task, this, std::placeholders::_1, std::placeholders::_2));

  status_gate_sub_ = create_subscription<RobotStatusMsg>(
    "robot_status",
    rclcpp::QoS(10),
    std::bind(&TaskManagerNode::on_robot_status_for_gate, this, std::placeholders::_1));

  localize_sub_ = create_subscription<custom_msgs_srvs::msg::LocalizeNavCommand>(
    "localize_nav_command",
    rclcpp::QoS(10),
    std::bind(&TaskManagerNode::on_localize_nav, this, std::placeholders::_1));

  initial_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initial_pose_topic_, rclcpp::QoS(10));

  auto task_qos = rclcpp::QoS(10).reliable().transient_local();
  task_info_sub_ = create_subscription<custom_msgs_srvs::msg::TaskInfo>(
    "task_info", rclcpp::QoS(10),
    std::bind(&TaskManagerNode::on_task_info, this, std::placeholders::_1));
  task_command_sub_ = create_subscription<custom_msgs_srvs::msg::TaskCommand>(
    "task_command", rclcpp::QoS(10),
    std::bind(&TaskManagerNode::on_task_command, this, std::placeholders::_1));
  navigation_status_sub_ = create_subscription<custom_msgs_srvs::msg::TaskStatus>(
    "navigation/task_status", task_qos,
    std::bind(&TaskManagerNode::on_navigation_status, this, std::placeholders::_1));
  navigation_info_pub_ = create_publisher<custom_msgs_srvs::msg::TaskInfo>(
    "navigation/task_info", rclcpp::QoS(10));
  navigation_command_pub_ = create_publisher<custom_msgs_srvs::msg::TaskCommand>(
    "navigation/task_command", rclcpp::QoS(10));
  elevator_status_sub_ = create_subscription<custom_msgs_srvs::msg::ElevatorStatus>(
    "fake_elevator/status", task_qos,
    std::bind(&TaskManagerNode::on_elevator_status, this, std::placeholders::_1));
  elevator_info_pub_ = create_publisher<custom_msgs_srvs::msg::ElevatorInfo>(
    "fake_elevator/info", rclcpp::QoS(10));
  elevator_command_pub_ = create_publisher<custom_msgs_srvs::msg::ElevatorCommand>(
    "fake_elevator/command", rclcpp::QoS(10));
  task_status_pub_ = create_publisher<custom_msgs_srvs::msg::TaskStatus>(
    "task_status", task_qos);
  publish_task_status(custom_msgs_srvs::msg::TaskStatus::STATUS_WAITING, "waiting for task");

  RCLCPP_INFO(
    get_logger(),
    "task_manager: task_info/task_command -> navigation task API; legacy set_robot_task and "
    "localize_nav_command retained; pub initial on %s",
    initial_pose_topic_.c_str());
}

void TaskManagerNode::publish_task_status(
  const std::string & status, const std::string & message) {
  custom_msgs_srvs::msg::TaskStatus output;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    output = active_status_;
    output.header.stamp = now();
    output.header.frame_id = "map";
    output.task_id = has_active_task_ ? active_task_.task_id : "";
    output.task_status = status;
    output.message = message;
    active_status_ = output;
  }
  if (task_status_pub_) {
    task_status_pub_->publish(output);
  }
}

void TaskManagerNode::on_task_info(
  const custom_msgs_srvs::msg::TaskInfo::SharedPtr msg) {
  const std::string id = strip(msg->task_id);
  const std::string type = to_lower(strip(msg->task_type));
  const std::string end = to_lower(strip(msg->end_action));
  std::string error;
  if (id.empty()) {
    error = "task_id is required";
  } else if (type != "navigation" && type != "patrol" && type != "following") {
    error = "task_type must be navigation, patrol, or following";
  } else if (!end.empty() && end != "waiting" && end != "back") {
    error = "end_action must be waiting or back";
  } else if (msg->poses.empty()) {
    error = "poses must not be empty";
  } else if (!msg->floor_ids.empty() && msg->floor_ids.size() != msg->poses.size()) {
    error = "floor_ids must be empty or match poses length";
  } else if (!msg->floor_ids.empty() && std::any_of(
      msg->floor_ids.begin(), msg->floor_ids.end(),
      [](const std::string & floor) {return strip(floor).empty();}))
  {
    error = "floor_ids entries must not be empty";
  } else if (end == "back" && msg->poses.size() < 2u) {
    error = "end_action=back requires the final pose to be the return pose";
  } else if (type == "navigation" && msg->poses.size() > 2u) {
    error = "navigation task accepts one goal and one optional return pose";
  }
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (has_active_task_ && !terminal_task_status(active_status_.task_status) &&
      id == active_task_.task_id)
    {
      RCLCPP_WARN(get_logger(), "duplicate active task ignored: %s", id.c_str());
      return;
    }
    active_task_ = *msg;
    active_task_.task_id = id;
    active_task_.task_type = type;
    active_task_.end_action = end.empty() ? "waiting" : end;
    has_active_task_ = true;
    active_status_ = custom_msgs_srvs::msg::TaskStatus();
    active_status_.task_id = id;
    work_items_.clear();
    current_work_index_ = 0;

    if (error.empty() && msg->floor_ids.empty()) {
      WorkItem item;
      item.kind = "navigation";
      item.label = "navigation:goal";
      item.navigation = active_task_;
      work_items_.push_back(item);
    } else if (error.empty()) {
      std::string current_floor = last_current_map_;
      auto append_navigation_pose = [&](const geometry_msgs::msg::Pose & pose,
          const std::string & floor, const std::string & label) {
          WorkItem item;
          item.kind = "navigation";
          item.label = label;
          item.navigation = active_task_;
          item.navigation.task_type = "navigation";
          item.navigation.end_action = "waiting";
          item.navigation.poses = {pose};
          item.navigation.floor_ids = {floor};
          work_items_.push_back(item);
        };
      auto append_elevator = [&](const std::string & operation,
          const std::string & from_floor, const std::string & target_floor,
          const ElevatorPoints & source, const ElevatorPoints & target) {
          WorkItem item;
          item.kind = "fake_elevator";
          item.label = "fake_elevator:" + operation + ":" +
            (operation == "call" ? from_floor : target_floor);
          item.elevator.header = active_task_.header;
          item.elevator.task_id = id;
          item.elevator.operation = operation;
          item.elevator.from_floor = from_floor;
          item.elevator.target_floor = target_floor;
          if (operation == "ride") {
            item.elevator.relocalization_pose.header = active_task_.header;
            item.elevator.relocalization_pose.header.frame_id = "map";
            item.elevator.relocalization_pose.pose.pose = source.inside;
            item.elevator.use_pose_first = true;
            item.elevator.target_inside_pose = target.inside;
          }
          work_items_.push_back(item);
        };
      size_t begin = 0;
      bool starts_inside_elevator = is_elevator_position(last_current_position_);
      while (begin < msg->poses.size()) {
        const std::string target_floor = strip(msg->floor_ids[begin]);
        size_t end_index = begin + 1;
        while (end_index < msg->poses.size() &&
          strip(msg->floor_ids[end_index]) == target_floor)
        {
          ++end_index;
        }
        if (target_floor != current_floor) {
          if (current_floor.empty()) {
            error = "current floor is unknown; wait for RobotStatus before cross-floor task";
            break;
          }
          ElevatorPoints source_points;
          ElevatorPoints target_points;
          if (!load_elevator_points(map_root_, current_floor, &source_points, &error) ||
            !load_elevator_points(map_root_, target_floor, &target_points, &error))
          {
            break;
          }
          if (!starts_inside_elevator) {
            append_navigation_pose(
              source_points.waiting, current_floor,
              "navigation:elevator_waiting:" + current_floor);
            append_elevator("call", current_floor, current_floor, source_points, source_points);
            append_navigation_pose(
              source_points.inside, current_floor,
              "navigation:elevator_inside:" + current_floor);
          }
          append_elevator("ride", current_floor, target_floor, source_points, target_points);
          append_navigation_pose(
            reverse_pose(target_points.waiting), target_floor,
            "navigation:elevator_exit:" + target_floor);
          starts_inside_elevator = false;
        }
        WorkItem navigation;
        navigation.kind = "navigation";
        navigation.label = "navigation:goal:" + target_floor;
        navigation.navigation = active_task_;
        navigation.navigation.poses.assign(
          msg->poses.begin() + static_cast<std::ptrdiff_t>(begin),
          msg->poses.begin() + static_cast<std::ptrdiff_t>(end_index));
        navigation.navigation.floor_ids.assign(
          msg->floor_ids.begin() + static_cast<std::ptrdiff_t>(begin),
          msg->floor_ids.begin() + static_cast<std::ptrdiff_t>(end_index));
        navigation.navigation.end_action = "waiting";
        work_items_.push_back(navigation);
        if (!target_floor.empty()) current_floor = target_floor;
        begin = end_index;
      }
    }

    active_status_.total_count = work_items_.size();
    active_status_.current_index = 0;
    active_status_.work_queue.clear();
    active_status_.model_status.clear();
    for (const auto & item : work_items_) {
      active_status_.work_queue.push_back(item.label);
    }
    active_status_.model_status.assign(
      active_status_.work_queue.size(), custom_msgs_srvs::msg::TaskStatus::STATUS_WAITING);
  }
  if (!error.empty()) {
    publish_task_status(custom_msgs_srvs::msg::TaskStatus::STATUS_FAILED, error);
    return;
  }
  publish_task_status(custom_msgs_srvs::msg::TaskStatus::STATUS_WAITING, "task accepted");
  dispatch_current_work();
}

void TaskManagerNode::dispatch_current_work() {
  WorkItem item;
  bool finished = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_active_task_) return;
    if (current_work_index_ >= work_items_.size()) {
      finished = true;
    } else {
      item = work_items_[current_work_index_];
      active_status_.current_index = current_work_index_;
      active_status_.model_status[current_work_index_] =
        custom_msgs_srvs::msg::TaskStatus::STATUS_WAITING;
    }
  }
  if (finished) {
    publish_task_status(custom_msgs_srvs::msg::TaskStatus::STATUS_FINISHED, "task finished");
  } else if (item.kind == "fake_elevator") {
    elevator_info_pub_->publish(item.elevator);
  } else {
    navigation_info_pub_->publish(item.navigation);
  }
}

void TaskManagerNode::on_task_command(
  const custom_msgs_srvs::msg::TaskCommand::SharedPtr msg) {
  const std::string command = to_lower(strip(msg->command));
  std::string kind;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const bool forward = has_active_task_ && strip(msg->task_id) == active_task_.task_id &&
      (command == "pause" || command == "resume" || command == "terminate") &&
      !terminal_task_status(active_status_.task_status);
    if (forward && current_work_index_ < work_items_.size()) {
      kind = work_items_[current_work_index_].kind;
    }
  }
  if (kind.empty()) {
    RCLCPP_WARN(get_logger(), "task command ignored: id=%s command=%s",
      msg->task_id.c_str(), msg->command.c_str());
    return;
  }
  if (kind == "fake_elevator") {
    custom_msgs_srvs::msg::ElevatorCommand command_msg;
    command_msg.header = msg->header;
    command_msg.task_id = msg->task_id;
    command_msg.command = msg->command;
    elevator_command_pub_->publish(command_msg);
  } else {
    navigation_command_pub_->publish(*msg);
  }
}

void TaskManagerNode::on_navigation_status(
  const custom_msgs_srvs::msg::TaskStatus::SharedPtr msg) {
  std::string root_status;
  custom_msgs_srvs::msg::TaskStatus output;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_active_task_ || msg->task_id != active_task_.task_id ||
      current_work_index_ >= work_items_.size() ||
      work_items_[current_work_index_].kind != "navigation") return;
    active_status_.current_index = current_work_index_;
    active_status_.model_status[current_work_index_] = msg->task_status;
    root_status = msg->task_status;
    if (active_task_.task_type == "patrol" &&
      msg->task_status == custom_msgs_srvs::msg::TaskStatus::STATUS_NAVIGATING &&
      (current_work_index_ > 0 || msg->current_index > 0))
    {
      root_status = custom_msgs_srvs::msg::TaskStatus::STATUS_PATROLLING;
    } else if (active_task_.task_type == "following" &&
      msg->task_status == custom_msgs_srvs::msg::TaskStatus::STATUS_FOLLOWING)
    {
      root_status = custom_msgs_srvs::msg::TaskStatus::STATUS_CLEANING;
    }
    active_status_.header.stamp = now();
    active_status_.header.frame_id = "map";
    active_status_.task_status = root_status;
    active_status_.message = msg->message;
    if (msg->task_status == custom_msgs_srvs::msg::TaskStatus::STATUS_FINISHED) {
      active_status_.model_status[current_work_index_] =
        custom_msgs_srvs::msg::TaskStatus::STATUS_FINISHED;
      ++current_work_index_;
      active_status_.current_index = current_work_index_;
      if (current_work_index_ < work_items_.size()) {
        active_status_.task_status = custom_msgs_srvs::msg::TaskStatus::STATUS_WAITING;
        active_status_.message = "dispatching next work item";
      }
    }
    output = active_status_;
  }
  task_status_pub_->publish(output);
  if (msg->task_status == custom_msgs_srvs::msg::TaskStatus::STATUS_FINISHED) {
    dispatch_current_work();
  }
}

void TaskManagerNode::on_elevator_status(
  const custom_msgs_srvs::msg::ElevatorStatus::SharedPtr msg) {
  custom_msgs_srvs::msg::TaskStatus output;
  bool advance = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_active_task_ || msg->task_id != active_task_.task_id ||
      current_work_index_ >= work_items_.size() ||
      work_items_[current_work_index_].kind != "fake_elevator") return;
    const std::string module_status = msg->elevator_status;
    active_status_.model_status[current_work_index_] = module_status;
    active_status_.header.stamp = now();
    active_status_.header.frame_id = "map";
    active_status_.message = msg->message;
    if (module_status == custom_msgs_srvs::msg::ElevatorStatus::STATUS_CALLING) {
      active_status_.task_status =
        custom_msgs_srvs::msg::TaskStatus::STATUS_CALLING_ELEVATOR;
    } else if (module_status == custom_msgs_srvs::msg::ElevatorStatus::STATUS_ENTERING) {
      active_status_.task_status =
        custom_msgs_srvs::msg::TaskStatus::STATUS_ELEVATOR_ENTERING;
    } else if (module_status == custom_msgs_srvs::msg::ElevatorStatus::STATUS_RIDING) {
      active_status_.task_status =
        custom_msgs_srvs::msg::TaskStatus::STATUS_RIDING_ELEVATOR;
    } else if (module_status == custom_msgs_srvs::msg::ElevatorStatus::STATUS_MOVING_MODEL) {
      active_status_.task_status = custom_msgs_srvs::msg::TaskStatus::STATUS_MOVING_MODEL;
    } else if (module_status == custom_msgs_srvs::msg::ElevatorStatus::STATUS_SWITCHING_MAP) {
      active_status_.task_status = custom_msgs_srvs::msg::TaskStatus::STATUS_SWITCHING_MAP;
    } else if (module_status == custom_msgs_srvs::msg::ElevatorStatus::STATUS_RELOCALIZING) {
      active_status_.task_status = custom_msgs_srvs::msg::TaskStatus::STATUS_RELOCALIZING;
    } else {
      active_status_.task_status = module_status;
    }
    if (module_status == custom_msgs_srvs::msg::ElevatorStatus::STATUS_FINISHED) {
      ++current_work_index_;
      active_status_.current_index = current_work_index_;
      advance = true;
      if (current_work_index_ < work_items_.size()) {
        active_status_.task_status = custom_msgs_srvs::msg::TaskStatus::STATUS_WAITING;
        active_status_.message = "dispatching next work item";
      }
    }
    output = active_status_;
  }
  task_status_pub_->publish(output);
  if (advance) dispatch_current_work();
}

void TaskManagerNode::set_self(std::shared_ptr<TaskManagerNode> self) {
  self_ = std::move(self);
}

void TaskManagerNode::on_robot_status_for_gate(const RobotStatusMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  last_robot_status_ = to_lower(strip(msg->robot_status));
  last_task_status_ = to_lower(strip(msg->task_status));
  last_current_map_ = strip(msg->current_map);
  last_current_position_ = strip(msg->current_position);
}

bool TaskManagerNode::send_heartbeat(
  const std::string & robot_status,
  const std::string & task_status,
  const std::string & current_map) {
  if (!hb_client_->wait_for_service(std::chrono::milliseconds(800))) {
    return false;
  }
  auto req = std::make_shared<Srv::Request>();
  req->robot_name = "";
  req->robot_status = robot_status;
  req->task_status = task_status;
  req->current_map = current_map;
  req->rate_hz = 0.0;
  req->task_progress = -1.0;
  auto fut = hb_client_->async_send_request(req);
  if (!self_) {
    return false;
  }
  if (!wait_client_future_no_spin(fut, std::chrono::seconds(8))) {
    return false;
  }
  const auto res = fut.get();
  return res && res->success;
}

void TaskManagerNode::on_set_task(
  const std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Request> req,
  std::shared_ptr<custom_msgs_srvs::srv::SetRobotTask::Response> res) {
  const std::string t = to_lower(strip(req->task_status));
  if (!valid_task_status(t)) {
    res->success = false;
    res->message = "task_status must be idle|mapping|delivery|cleaning|patrolling";
    return;
  }
  if (!hb_client_->wait_for_service(std::chrono::seconds(2))) {
    res->success = false;
    res->message = "set_heartbeat_params not available";
    return;
  }
  auto hb_req = std::make_shared<Srv::Request>();
  const std::string rs = to_lower(strip(req->robot_status));
  if (rs.empty()) {
    hb_req->robot_status = "";
  } else if (valid_robot_status(rs)) {
    hb_req->robot_status = rs;
  } else {
    res->success = false;
    res->message =
      "robot_status must be empty (leave) or initializing|localizing|localization_lost|ready|shutdown";
    return;
  }
  hb_req->task_status = t;
  hb_req->robot_name = "";
  hb_req->rate_hz = 0.0;
  hb_req->task_progress = -1.0;
  auto future = hb_client_->async_send_request(hb_req);
  if (!self_) {
    res->success = false;
    res->message = "task_manager not fully initialized";
    return;
  }
  if (!wait_client_future_no_spin(future, std::chrono::seconds(8))) {
    res->success = false;
    res->message = "heartbeat call timeout";
    return;
  }
  const auto & hb_res = future.get();
  res->success = hb_res->success;
  res->message = hb_res->message;
}

void TaskManagerNode::on_localize_nav(
  const custom_msgs_srvs::msg::LocalizeNavCommand::SharedPtr msg) {
  const std::string want = leaf_namespace_id(get_namespace());
  const std::string rid = strip(msg->robot_id);
  if (rid.empty() || rid != want) {
    RCLCPP_WARN(
      get_logger(),
      "localize_nav_command ignored: robot_id %s != namespace id %s",
      rid.c_str(), want.c_str());
    return;
  }

  std::string rs;
  std::string ts;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    rs = last_robot_status_;
    ts = last_task_status_;
  }

  if (ts == "mapping") {
    RCLCPP_WARN(get_logger(), "localize_nav_command ignored: task_status is mapping");
    return;
  }
  if (rs == "shutdown") {
    RCLCPP_WARN(get_logger(), "localize_nav_command ignored: robot_status is shutdown");
    return;
  }
  // localization_lost is the post-relocate resting state until pose recovers; must accept
  // further Web reloc clicks (otherwise only the first relocate works).
  if (rs != "initializing" && rs != "localizing" && rs != "localization_lost" && rs != "ready") {
    RCLCPP_WARN(
      get_logger(),
      "localize_nav_command ignored: robot_status=%s not in {initializing,localizing,localization_lost,ready}",
      rs.c_str());
    return;
  }

  const std::string map = strip(msg->map_name);
  if (map.empty() && !msg->set_initial_pose) {
    RCLCPP_WARN(get_logger(), "localize_nav_command ignored: empty map_name and no pose");
    return;
  }

  // The localization pose is authoritative. AMCL replaces its old
  // localization buffer and processes the next scan around this pose.
  if (msg->set_initial_pose && initial_pub_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    // Use the latest available TF. Stamping this with now() can put the pose a
    // fraction ahead of odom->base_footprint and make AMCL reject an otherwise
    // valid relocation with "extrapolation into the future".
    pose.header.stamp.sec = 0;
    pose.header.stamp.nanosec = 0;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = msg->x;
    pose.pose.pose.position.y = msg->y;
    pose.pose.pose.position.z = 0.0;
    const double yaw = msg->yaw;
    pose.pose.pose.orientation.x = 0.0;
    pose.pose.pose.orientation.y = 0.0;
    pose.pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.pose.orientation.w = std::cos(yaw / 2.0);
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0] = 0.25;
    pose.pose.covariance[7] = 0.25;
    pose.pose.covariance[35] = 0.068;
    initial_pub_->publish(pose);
    RCLCPP_INFO(
      get_logger(),
      "localize_nav: published initial pose x=%.3f y=%.3f yaw=%.4f",
      msg->x, msg->y, yaw);
  }

  // State publication is best-effort metadata and happens after the pose so
  // service delays or failures cannot postpone localization.
  if (!send_heartbeat("", "idle", "")) {
    RCLCPP_WARN(get_logger(), "localize_nav: failed to set task idle; continuing");
  }
  if (!map.empty()) {
    if (!send_heartbeat("", "", map)) {
      RCLCPP_WARN(get_logger(), "localize_nav: failed to set current_map; continuing");
    }
  }
  if (!send_heartbeat("localization_lost", "", "")) {
    RCLCPP_WARN(
      get_logger(),
      "localize_nav: failed to set robot_status localization_lost; continuing");
  }

  RCLCPP_INFO(get_logger(), "localize_nav_command applied for %s", rid.c_str());
}

}  // namespace manager

#ifndef MANAGER_TASK_MANAGER_NO_MAIN
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manager::TaskManagerNode>();
  node->set_self(node);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4u);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
