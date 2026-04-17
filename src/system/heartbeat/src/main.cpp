#include <memory>

#include "heartbeat/heartbeat_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<heartbeat::HeartbeatNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
