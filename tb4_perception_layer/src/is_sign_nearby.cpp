#include "tb4_perception_layer/is_sign_nearby.hpp"

#include <cmath>
#include <sstream>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "tf2_ros/buffer.h"

namespace tb4_perception_layer
{

IsSignNearby::IsSignNearby(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
}

BT::NodeStatus IsSignNearby::tick()
{
  if (!initialized_) {
    // Get the shared ROS node from the BT blackboard
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    double dist = 2.0;
    getInput("distance_threshold", dist);
    distance_threshold_ = dist;

    double rearm = 1.5;
    getInput("rearm_clear_time", rearm);
    rearm_clear_time_ = rearm;

    std::string topic = "/semantic_obstacles";
    getInput("semantic_topic", topic);

    // Sign classes that trigger the 2 s stop (both STOP and DO NOT ENTER stop;
    // the drive-through vs reroute difference comes from the costmap keep-out).
    std::string classes_csv = "stop sign,do not enter";
    getInput("trigger_classes", classes_csv);
    std::stringstream cs(classes_csv);
    std::string item;
    while (std::getline(cs, item, ',')) {
      size_t a = item.find_first_not_of(" \t");
      size_t b = item.find_last_not_of(" \t");
      if (a != std::string::npos) {
        trigger_classes_.insert(item.substr(a, b - a + 1));
      }
    }

    // BEST_EFFORT to match the fusion publisher's effective delivery across the
    // Fast-DDS discovery-server/async-writer link: a RELIABLE reader gets no
    // cross-machine delivery here, while the BEST_EFFORT costmap readers do.
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    sub_ = node_->create_subscription<msg::SemanticObstacleArray>(
      topic, qos,
      std::bind(&IsSignNearby::obstacleCallback, this,
        std::placeholders::_1),
      sub_option);

    RCLCPP_INFO(node_->get_logger(),
      "IsSignNearby: listening on '%s', threshold=%.1f m, rearm=%.1f s, "
      "classes=[%s]",
      topic.c_str(), distance_threshold_, rearm_clear_time_, classes_csv.c_str());

    initialized_ = true;
  }

  // Service our own subscription — the BT's shared node executor does not.
  callback_group_executor_.spin_some();

  std::lock_guard<std::mutex> lock(mutex_);

  // Debounced "nearby": true if a trigger sign was seen within the threshold
  // recently (within rearm_clear_time_ seconds). This absorbs detection
  // flicker so the latch does not re-fire spuriously.
  const rclcpp::Time now = node_->get_clock()->now();
  const bool nearby =
    (now - last_nearby_stamp_).seconds() < rearm_clear_time_;

  if (!nearby) {
    // Sign is gone (or was never seen): re-arm for the next detection.
    armed_ = true;
    return BT::NodeStatus::FAILURE;
  }

  if (armed_) {
    // Rising edge — fire exactly once, then suppress until re-armed.
    armed_ = false;
    RCLCPP_WARN(node_->get_logger(),
      "Sign detected nearby! Triggering single hard stop + replan.");
    return BT::NodeStatus::SUCCESS;
  }

  // Still nearby but already fired for this detection episode: suppress.
  return BT::NodeStatus::FAILURE;
}

void IsSignNearby::obstacleCallback(
  const tb4_perception_layer::msg::SemanticObstacleArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  double robot_x = 0.0, robot_y = 0.0;
  std::string obs_frame = msg->header.frame_id;
  try {
    auto tf_buffer = config().blackboard->get<
      std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    if (tf_buffer) {
      if (obs_frame.empty()) {
        obs_frame = "map";
        getInput("global_frame", obs_frame);
      }

      auto transform = tf_buffer->lookupTransform(
        obs_frame, "base_link", tf2::TimePointZero);
      robot_x = transform.transform.translation.x;
      robot_y = transform.transform.translation.y;
    }
  } catch (const std::exception &) {
    // If TF fails, use (0,0) — conservative fallback.
  }

  for (const auto & obs : msg->obstacles) {
    if (trigger_classes_.find(obs.class_id) == trigger_classes_.end()) {
      continue;
    }
    double dx = obs.x - robot_x;
    double dy = obs.y - robot_y;
    double dist = std::hypot(dx, dy);
    if (dist <= distance_threshold_) {
      // Refresh the "last seen nearby" timestamp used by the debounce.
      last_nearby_stamp_ = node_->get_clock()->now();
      return;
    }
  }
}

}  // namespace tb4_perception_layer

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tb4_perception_layer::IsSignNearby>(
    "IsSignNearby");
}
