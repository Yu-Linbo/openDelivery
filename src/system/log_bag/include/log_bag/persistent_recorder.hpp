#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>

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
    std::shared_ptr<Serialized> latest_image;
    std::chrono::steady_clock::time_point latest_image_received{};
  };

public:
  PersistentRecorder(rclcpp::Node::SharedPtr node, const std::vector<std::string> & topics)
  : node_(std::move(node)), wanted_(topics.begin(), topics.end()) {}

  bool active() const {return bool(writer_);}
  void close() {writer_.reset();}

  // Callbacks share one SingleThreadedExecutor, so each cached frame stays
  // stable while a task boundary writes it. Missing frames are simply skipped.
  std::size_t snapshot_images() {
    if (!writer_) {return 0;}
    const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto now = std::chrono::steady_clock::now();
    std::size_t written = 0;
    for (const auto & entry : topics_) {
      if (entry.second.latest_image &&
        now - entry.second.latest_image_received <= std::chrono::seconds(2))
      {
        write(entry.first, entry.second.latest_image, timestamp);
        ++written;
      }
    }
    return written;
  }

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
    for (auto & entry : topics_) {
      if (entry.second.metadata.type == "custom_msgs_srvs/msg/TaskStatus") {
        // A restarted task manager has a new publisher GID. Its old retained
        // status must not be copied into later bags beside the new one.
        std::set<std::string> live_publishers;
        for (const auto & publisher : node_->get_publishers_info_by_topic(entry.first)) {
          const auto & gid = publisher.endpoint_gid();
          live_publishers.emplace(reinterpret_cast<const char *>(gid.data()), gid.size());
        }
        for (auto it = entry.second.retained.begin(); it != entry.second.retained.end();) {
          if (live_publishers.count(it->first) == 0) {
            it = entry.second.retained.erase(it);
          } else {
            ++it;
          }
        }
      }
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
      const bool image_topic = is_snapshot_image_topic(name);
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
      callback.set([this, name, retained, image_topic](
        std::shared_ptr<Serialized> message, const rclcpp::MessageInfo & info) {
          if (image_topic) {
            auto & topic = topics_.at(name);
            topic.latest_image = std::move(message);
            topic.latest_image_received = std::chrono::steady_clock::now();
            return;
          }
          if (retained) {
            auto & topic = topics_.at(name);
            // Root TaskStatus is a single current state. A restarted manager
            // has a different GID; its new state supersedes every old one.
            if (topic.metadata.type == "custom_msgs_srvs/msg/TaskStatus") {
              topic.retained.clear();
            }
            const auto & gid = info.get_rmw_message_info().publisher_gid;
            topic.retained[std::string(
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
  static bool is_snapshot_image_topic(const std::string & name) {
    for (const char * suffix : {
        "/front_camera/image_raw", "/front_down_camera/image_raw"})
    {
      const std::string ending(suffix);
      if (name.size() >= ending.size() &&
        name.compare(name.size() - ending.size(), ending.size(), ending) == 0)
      {
        return true;
      }
    }
    return false;
  }

  void write(const std::string & topic, const std::shared_ptr<Serialized> & message,
    std::int64_t timestamp = 0) {
    if (!writer_) {return;}
    auto bag = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    // Alias the buffer while keeping the owning SerializedMessage alive.
    bag->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      message, &message->get_rcl_serialized_message());
    bag->topic_name = topic;
    bag->time_stamp = timestamp ? timestamp : std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    writer_->write(bag);
  }

  rclcpp::Node::SharedPtr node_;
  std::set<std::string> wanted_;
  std::map<std::string, Topic> topics_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

}  // namespace log_bag
