#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ManagedSlamLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ManagedSlamLifecycleNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("managed_slam_lifecycle", options)
  {
    // LifecycleNode already exposes use_sim_time in many ROS2 setups.
    // Guard redeclaration to avoid ParameterAlreadyDeclaredException.
    if (!has_parameter("use_sim_time")) {
      declare_parameter<bool>("use_sim_time", true);
    }
    declare_parameter<std::string>("slam_executable", "sync_slam_toolbox_node");
    declare_parameter<std::string>("child_node_name", "mapping_worker");
    declare_parameter<std::string>("child_namespace", "/");
    declare_parameter<std::string>("params_file", "");
    declare_parameter<std::string>("mode", "mapping");
    declare_parameter<std::string>("map_file_name", "");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<std::string>("scan_topic", "/scan_2d");
    declare_parameter<std::string>("map_name", "/map");
  }

  ~ManagedSlamLifecycleNode() override
  {
    stop_child_process();
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    get_parameter("slam_executable", slam_executable_);
    get_parameter("child_node_name", child_node_name_);
    get_parameter("child_namespace", child_namespace_);
    get_parameter("params_file", params_file_);
    get_parameter("use_sim_time", use_sim_time_);
    get_parameter("mode", mode_);
    get_parameter("map_file_name", map_file_name_);
    get_parameter("map_frame", map_frame_);
    get_parameter("odom_frame", odom_frame_);
    get_parameter("base_frame", base_frame_);
    get_parameter("scan_topic", scan_topic_);
    get_parameter("map_name", map_name_);

    if (params_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "params_file is empty");
      return CallbackReturn::FAILURE;
    }

    try {
      slam_toolbox_exec_path_ =
        ament_index_cpp::get_package_prefix("slam_toolbox") + "/lib/slam_toolbox/" + slam_executable_;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve slam_toolbox path: %s", e.what());
      return CallbackReturn::FAILURE;
    }

    if (access(slam_toolbox_exec_path_.c_str(), X_OK) != 0) {
      RCLCPP_ERROR(get_logger(), "Executable not found or not executable: %s", slam_toolbox_exec_path_.c_str());
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(
      get_logger(),
      "Configured managed slam lifecycle node. executable=%s child=%s ns=%s",
      slam_executable_.c_str(), child_node_name_.c_str(), child_namespace_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (child_pid_ > 0) {
      RCLCPP_INFO(get_logger(), "SLAM child already running pid=%d", child_pid_);
      return CallbackReturn::SUCCESS;
    }
    if (!start_child_process()) {
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    stop_child_process();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    stop_child_process();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    stop_child_process();
    return CallbackReturn::SUCCESS;
  }

private:
  bool start_child_process()
  {
    // Empty "-p key:=" is invalid on Foxy (RCLInvalidROSArgsError); mapping mode has no map_file_name.
    std::vector<std::string> args = {
      slam_toolbox_exec_path_,
      "--ros-args",
      "-r", "__node:=" + child_node_name_,
      "-r", "__ns:=" + child_namespace_,
      "--params-file", params_file_,
      "-p", "use_sim_time:=" + std::string(use_sim_time_ ? "true" : "false"),
      "-p", "mode:=" + mode_,
    };
    if (!map_file_name_.empty()) {
      args.push_back("-p");
      args.push_back("map_file_name:=" + map_file_name_);
    }
    args.push_back("-p");
    args.push_back("map_frame:=" + map_frame_);
    args.push_back("-p");
    args.push_back("odom_frame:=" + odom_frame_);
    args.push_back("-p");
    args.push_back("base_frame:=" + base_frame_);
    args.push_back("-p");
    args.push_back("scan_topic:=" + scan_topic_);
    args.push_back("-p");
    args.push_back("map_name:=" + map_name_);

    std::vector<char *> argv;
    argv.reserve(args.size() + 1);
    for (auto & item : args) {
      argv.push_back(const_cast<char *>(item.c_str()));
    }
    argv.push_back(nullptr);

    pid_t pid = fork();
    if (pid < 0) {
      RCLCPP_ERROR(get_logger(), "fork() failed");
      return false;
    }

    if (pid == 0) {
      execv(argv[0], argv.data());
      _exit(127);
    }

    child_pid_ = pid;
    RCLCPP_INFO(get_logger(), "Started SLAM child process pid=%d executable=%s", child_pid_, slam_executable_.c_str());
    return true;
  }

  void stop_child_process()
  {
    if (child_pid_ <= 0) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Stopping SLAM child process pid=%d", child_pid_);
    kill(child_pid_, SIGINT);

    constexpr int max_wait_cycles = 50;
    for (int i = 0; i < max_wait_cycles; ++i) {
      int status = 0;
      pid_t ret = waitpid(child_pid_, &status, WNOHANG);
      if (ret == child_pid_) {
        child_pid_ = -1;
        return;
      }
      if (ret < 0) {
        child_pid_ = -1;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    kill(child_pid_, SIGKILL);
    waitpid(child_pid_, nullptr, 0);
    child_pid_ = -1;
  }

  pid_t child_pid_{-1};

  std::string slam_toolbox_exec_path_;
  std::string slam_executable_;
  std::string child_node_name_;
  std::string child_namespace_;
  std::string params_file_;
  bool use_sim_time_{true};
  std::string mode_;
  std::string map_file_name_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string map_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagedSlamLifecycleNode>(rclcpp::NodeOptions{});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
