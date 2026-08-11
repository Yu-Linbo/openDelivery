#ifndef MANAGER__NODE_PING_CHECKER_HPP_
#define MANAGER__NODE_PING_CHECKER_HPP_

#include <string>
#include <exception>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace manager {

// ROS graph reachability check.  It intentionally does not measure latency.
class NodePingChecker {
public:
  explicit NodePingChecker(rclcpp::Node * node) : node_(node) {}

  bool all_present(const std::vector<std::string> & required) const {
    if (required.empty() || node_ == nullptr) {
      return true;
    }
    std::vector<std::pair<std::string, std::string>> nodes;
    try {
      nodes = node_->get_node_graph_interface()->get_node_names_and_namespaces();
    } catch (const std::exception &) {
      return false;
    }
    for (const auto & target : required) {
      const auto resolved = resolve(target);
      if (resolved.second.empty()) {
        return false;
      }
      bool found = false;
      for (const auto & item : nodes) {
        if (item.first == resolved.second && item.second == resolved.first) {
          found = true;
          break;
        }
      }
      if (!found) {
        return false;
      }
    }
    return true;
  }

private:
  std::pair<std::string, std::string> resolve(const std::string & target) const {
    const auto last_slash = target.rfind('/');
    if (!target.empty() && target.front() == '/') {
      if (last_slash == 0 || last_slash == std::string::npos) {
        return {"/", target.substr(1)};
      }
      return {target.substr(0, last_slash), target.substr(last_slash + 1)};
    }
    const std::string base = node_->get_namespace() == "/" ? "" : node_->get_namespace();
    if (last_slash == std::string::npos) {
      return {base.empty() ? "/" : base, target};
    }
    return {base + "/" + target.substr(0, last_slash), target.substr(last_slash + 1)};
  }

  rclcpp::Node * node_;
};

}  // namespace manager

#endif
