#include "tb4_perception_layer/is_stop_sign_nearby.hpp"

#include <cmath>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "tf2_ros/buffer.h"

namespace tb4_perception_layer
{

IsStopSignNearby::IsStopSignNearby(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
}

BT::NodeStatus IsStopSignNearby::tick()
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

    sub_ = node_->create_subscription<msg::SemanticObstacleArray>(
      topic, 10,
      std::bind(&IsStopSignNearby::obstacleCallback, this,
        std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
      "IsStopSignNearby: listening on '%s', threshold=%.1f m, rearm=%.1f s",
      topic.c_str(), distance_threshold_, rearm_clear_time_);

    initialized_ = true;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Debounced "nearby": true if a stop sign was seen within the threshold
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
      "Stop sign detected nearby! Triggering single hard stop + replan.");
    return BT::NodeStatus::SUCCESS;
  }

  // Still nearby but already fired for this detection episode: suppress.
  return BT::NodeStatus::FAILURE;
}

void IsStopSignNearby::obstacleCallback(
  const tb4_perception_layer::msg::SemanticObstacleArray::SharedPtr msg)
{
  // Get robot position from TF via the blackboard. The obstacles carry their
  // own frame (the fusion node publishes in 'odom'), so look the robot up in
  // THAT frame — comparing an odom-frame obstacle to a map-frame robot pose
  // yields a wrong distance whenever map and odom are offset.
  double robot_x = 0.0, robot_y = 0.0;
  try {
    auto tf_buffer = config().blackboard->get<
      std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    if (tf_buffer) {
      std::string obs_frame = msg->header.frame_id;
      if (obs_frame.empty()) {
        obs_frame = "map";
        getInput("global_frame", obs_frame);
      }

      auto transform = tf_buffer->lookupTransform(
        obs_frame, "base_link", tf2::TimePointZero);
      robot_x = transform.transform.translation.x;
      robot_y = transform.transform.translation.y;
    }
  } catch (...) {
    // If TF fails, use (0,0) — conservative fallback
  }

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & obs : msg->obstacles) {
    if (obs.class_id != "stop sign") {
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
  factory.registerNodeType<tb4_perception_layer::IsStopSignNearby>(
    "IsStopSignNearby");
}
