#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <chrono>
#include <map>
#include <set>
#include <sstream>
#include <string>

namespace log_bag {

inline bool recording_start_allowed(const std::string & status) {
  return status == "localization_lost" || status == "ready";
}

// All methods/callbacks run on the recorder's single-threaded executor. Keep
// the DDS participant and subscriptions alive across file rotations.
class PersistentRecorder {
  using Serialized = rclcpp::SerializedMessage;
  struct Topic {
    rosbag2_storage::TopicMetadata metadata;
    std::shared_ptr<rcpputils::SharedLibrary> library;
    rclcpp::Subscription<Serialized>::SharedPtr subscription;
    std::map<std::string, std::shared_ptr<Serialized>> retained;
  };

public:
  PersistentRecorder(rclcpp::Node::SharedPtr node, const std::vector<std::string> & topics)
  : node_(std::move(node)), wanted_(topics.begin(), topics.end()) {}

  bool active() const {return bool(writer_);}
  void close() {writer_.reset();}

  void open(const std::string & path) {
    auto writer = std::make_unique<rosbag2_cpp::Writer>(
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
    rosbag2_cpp::StorageOptions storage;
    storage.uri = path;
    storage.storage_id = "sqlite3";
    writer->open(storage, {"cdr", "cdr"});
    for (const auto & entry : topics_) {
      writer->create_topic(entry.second.metadata);
    }
    writer_ = std::move(writer);
    // Every standalone bag needs latched TF/task context, even when the
    // publisher sent it only once before this rotation.
    for (const auto & entry : topics_) {
      for (const auto & sample : entry.second.retained) {
        write(entry.first, sample.second);
      }
    }
  }

  void discover() {
    for (const auto & entry : node_->get_topic_names_and_types()) {
      const auto & name = entry.first;
      if (!wanted_.count(name) || topics_.count(name) || entry.second.size() != 1) {
        continue;
      }
      const auto publishers = node_->get_publishers_info_by_topic(name);
      if (publishers.empty()) {continue;}
      // Match reliable publishers; accept best-effort sensor streams. Request
      // transient-local only when every current publisher offers it.
      auto qos = rclcpp::QoS(100);
      bool reliable = true;
      bool retained = true;
      std::ostringstream offered;
      for (const auto & publisher : publishers) {
        const auto & profile = publisher.qos_profile().get_rmw_qos_profile();
        offered << "- history: " << profile.history << "\n  depth: " << profile.depth
                << "\n  reliability: " << profile.reliability
                << "\n  durability: " << profile.durability;
        const auto duration = [&offered](const char * key, const rmw_time_t & value) {
            offered << "\n  " << key << ": {sec: " << value.sec
                    << ", nsec: " << value.nsec << "}";
          };
        duration("deadline", profile.deadline);
        duration("lifespan", profile.lifespan);
        offered << "\n  liveliness: " << profile.liveliness;
        duration("liveliness_lease_duration", profile.liveliness_lease_duration);
        offered << "\n  avoid_ros_namespace_conventions: "
                << (profile.avoid_ros_namespace_conventions ? "true" : "false") << "\n";
        reliable = reliable && publisher.qos_profile().get_rmw_qos_profile().reliability ==
          RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        retained = retained && publisher.qos_profile().get_rmw_qos_profile().durability ==
          RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      }
      if (!reliable) {qos.best_effort();}
      if (retained) {qos.transient_local();}
      Topic topic;
      topic.metadata = {name, entry.second.front(), "cdr", offered.str()};
      topic.library = rosbag2_cpp::get_typesupport_library(
        topic.metadata.type, "rosidl_typesupport_cpp");
      const auto * support = rosbag2_cpp::get_typesupport_handle(
        topic.metadata.type, "rosidl_typesupport_cpp", topic.library);
      rclcpp::AnySubscriptionCallback<Serialized, std::allocator<void>> callback(
        std::make_shared<std::allocator<void>>());
      callback.set([this, name, retained](
        std::shared_ptr<Serialized> message, const rclcpp::MessageInfo & info) {
          if (retained) {
            const auto & gid = info.get_rmw_message_info().publisher_gid;
            topics_.at(name).retained[std::string(
              reinterpret_cast<const char *>(gid.data), sizeof(gid.data))] = message;
          }
          write(name, message);
        });
      topic.subscription = std::make_shared<rclcpp::Subscription<Serialized>>(
        node_->get_node_base_interface().get(), *support, name, qos, callback,
        rclcpp::SubscriptionOptions(),
        rclcpp::message_memory_strategy::MessageMemoryStrategy<Serialized>::create_default());
      node_->get_node_topics_interface()->add_subscription(topic.subscription, nullptr);
      if (writer_) {writer_->create_topic(topic.metadata);}
      topics_.emplace(name, std::move(topic));
    }
  }

private:
  void write(const std::string & topic, const std::shared_ptr<Serialized> & message) {
    if (!writer_) {return;}
    auto bag = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    // Alias the buffer while keeping the owning SerializedMessage alive.
    bag->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      message, &message->get_rcl_serialized_message());
    bag->topic_name = topic;
    bag->time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    writer_->write(bag);
  }

  rclcpp::Node::SharedPtr node_;
  std::set<std::string> wanted_;
  std::map<std::string, Topic> topics_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

}  // namespace log_bag
