#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "gtest/gtest.h"

#include "manager/task_manager_node.hpp"

namespace manager {

class TaskManagerElevatorPlanTestPeer {
public:
  static void set_map_root(TaskManagerNode & node, const std::string & root) {
    node.map_root_ = root;
  }
  static void set_current_floor(TaskManagerNode & node, const std::string & floor) {
    auto status = std::make_shared<custom_msgs_srvs::msg::RobotStatus>();
    status->current_map = floor;
    node.on_robot_status_for_gate(status);
  }
  static void submit(TaskManagerNode & node, custom_msgs_srvs::msg::TaskInfo::SharedPtr task) {
    node.on_task_info(std::move(task));
  }
  static size_t size(const TaskManagerNode & node) {return node.work_items_.size();}
  static std::string label(const TaskManagerNode & node, size_t index) {
    return node.work_items_.at(index).label;
  }
  static custom_msgs_srvs::msg::ElevatorInfo elevator(
    const TaskManagerNode & node, size_t index)
  {
    return node.work_items_.at(index).elevator;
  }
  static custom_msgs_srvs::msg::TaskInfo navigation(
    const TaskManagerNode & node, size_t index)
  {
    return node.work_items_.at(index).navigation;
  }
  static custom_msgs_srvs::msg::TaskStatus status(const TaskManagerNode & node) {
    return node.active_status_;
  }
};

}  // namespace manager

namespace {

class ElevatorPlanTest : public ::testing::Test {
protected:
  void SetUp() override {
    int argc = 1;
    char program[] = "test_task_manager_elevator_plan";
    char * argv[] = {program, nullptr};
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(argc, argv);
    char path[] = "/tmp/open_delivery_elevator_plan_XXXXXX";
    root_ = ::mkdtemp(path);
    ASSERT_FALSE(root_.empty());
    write_floor("floor1", 1.0, 2.0, 0.2, 3.0, 4.0, 0.4);
    write_floor("floor2", 11.0, 12.0, 1.2, 13.0, 14.0, 1.4);
  }

  void TearDown() override {
    for (const auto & floor : {std::string("floor1"), std::string("floor2")}) {
      const std::string directory = root_ + "/" + floor;
      (void)::unlink((directory + "/" + floor + "_points.json").c_str());
      (void)::rmdir(directory.c_str());
    }
    (void)::rmdir(root_.c_str());
    context_->shutdown("test complete");
  }

  void write_floor(
    const std::string & floor, double inside_x, double inside_y, double inside_yaw,
    double waiting_x, double waiting_y, double waiting_yaw)
  {
    const std::string directory = root_ + "/" + floor;
    ASSERT_EQ(::mkdir(directory.c_str(), 0755), 0);
    std::ofstream out(directory + "/" + floor + "_points.json");
    out << "{\"points\":["
        << "{\"id\":\"inside\",\"type\":\"elevator_inside\",\"x\":" << inside_x
        << ",\"y\":" << inside_y << ",\"yaw\":" << inside_yaw << "},"
        << "{\"id\":\"waiting\",\"type\":\"elevator_waiting\",\"x\":" << waiting_x
        << ",\"y\":" << waiting_y << ",\"yaw\":" << waiting_yaw << "}]}";
  }

  std::string root_;
  std::shared_ptr<rclcpp::Context> context_;

  std::shared_ptr<manager::TaskManagerNode> make_node() {
    rclcpp::NodeOptions options;
    options.context(context_);
    return std::make_shared<manager::TaskManagerNode>(options);
  }
};

TEST_F(ElevatorPlanTest, ExpandsCrossFloorGoalIntoCompleteElevatorSequence) {
  auto node = make_node();
  manager::TaskManagerElevatorPlanTestPeer::set_map_root(*node, root_);
  manager::TaskManagerElevatorPlanTestPeer::set_current_floor(*node, "floor1");

  auto task = std::make_shared<custom_msgs_srvs::msg::TaskInfo>();
  task->task_id = "cross_floor";
  task->task_type = "navigation";
  task->end_action = "waiting";
  task->floor_ids = {"floor2"};
  geometry_msgs::msg::Pose goal;
  goal.position.x = 20.0;
  goal.position.y = 21.0;
  goal.orientation.w = 1.0;
  task->poses = {goal};
  manager::TaskManagerElevatorPlanTestPeer::submit(*node, task);

  ASSERT_EQ(manager::TaskManagerElevatorPlanTestPeer::size(*node), 6u);
  EXPECT_EQ(manager::TaskManagerElevatorPlanTestPeer::label(*node, 0), "navigation:elevator_waiting:floor1");
  EXPECT_EQ(manager::TaskManagerElevatorPlanTestPeer::label(*node, 1), "fake_elevator:call:floor1");
  EXPECT_EQ(manager::TaskManagerElevatorPlanTestPeer::label(*node, 2), "navigation:elevator_inside:floor1");
  EXPECT_EQ(manager::TaskManagerElevatorPlanTestPeer::label(*node, 3), "fake_elevator:ride:floor2");
  EXPECT_EQ(manager::TaskManagerElevatorPlanTestPeer::label(*node, 4), "navigation:elevator_exit:floor2");
  EXPECT_EQ(manager::TaskManagerElevatorPlanTestPeer::label(*node, 5), "navigation:goal:floor2");

  const auto call = manager::TaskManagerElevatorPlanTestPeer::elevator(*node, 1);
  EXPECT_EQ(call.operation, "call");
  const auto ride = manager::TaskManagerElevatorPlanTestPeer::elevator(*node, 3);
  EXPECT_EQ(ride.operation, "ride");
  EXPECT_TRUE(ride.use_pose_first);
  EXPECT_DOUBLE_EQ(ride.relocalization_pose.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(ride.relocalization_pose.pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(ride.target_inside_pose.position.x, 11.0);
  EXPECT_DOUBLE_EQ(ride.target_inside_pose.position.y, 12.0);

  const auto exit = manager::TaskManagerElevatorPlanTestPeer::navigation(*node, 4).poses.front();
  EXPECT_DOUBLE_EQ(exit.position.x, 13.0);
  EXPECT_DOUBLE_EQ(exit.position.y, 14.0);
  const double exit_yaw = 2.0 * std::atan2(exit.orientation.z, exit.orientation.w);
  EXPECT_NEAR(std::abs(std::remainder(
      exit_yaw - 1.4, 2.0 * 3.14159265358979323846)),
    3.14159265358979323846, 1e-9);
}

TEST_F(ElevatorPlanTest, MissingElevatorPointsFailsBeforeDispatch) {
  auto node = make_node();
  manager::TaskManagerElevatorPlanTestPeer::set_map_root(*node, root_);
  manager::TaskManagerElevatorPlanTestPeer::set_current_floor(*node, "floor1");
  std::ofstream(root_ + "/floor2/floor2_points.json") << "{\"points\":[]}";

  auto task = std::make_shared<custom_msgs_srvs::msg::TaskInfo>();
  task->task_id = "bad_cross_floor";
  task->task_type = "navigation";
  task->floor_ids = {"floor2"};
  geometry_msgs::msg::Pose goal;
  goal.orientation.w = 1.0;
  task->poses = {goal};
  manager::TaskManagerElevatorPlanTestPeer::submit(*node, task);

  const auto status = manager::TaskManagerElevatorPlanTestPeer::status(*node);
  EXPECT_EQ(status.task_status,
    custom_msgs_srvs::msg::TaskStatus::STATUS_FAILED);
  EXPECT_NE(status.message.find("requires exactly one"), std::string::npos);
}

}  // namespace
