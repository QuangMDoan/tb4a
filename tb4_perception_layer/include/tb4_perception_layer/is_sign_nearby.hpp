#ifndef TB4_PERCEPTION_LAYER__IS_SIGN_NEARBY_HPP_
#define TB4_PERCEPTION_LAYER__IS_SIGN_NEARBY_HPP_

#include <mutex>
#include <set>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tb4_perception_layer/msg/semantic_obstacle_array.hpp"

namespace tb4_perception_layer
{

/**
 * BT condition node that returns SUCCESS (once, edge-triggered) when any of
 * the configured sign classes (e.g. "stop sign", "do not enter") is detected
 * within a configurable distance of the robot, FAILURE otherwise.
 */
class IsSignNearby : public BT::ConditionNode
{
public:
  IsSignNearby(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance_threshold", 2.0,
        "Max distance (m) to consider a sign nearby"),
      BT::InputPort<std::string>("semantic_topic", "/semantic_obstacles",
        "SemanticObstacleArray topic"),
      BT::InputPort<std::string>("global_frame", "map",
        "Global frame for distance computation"),
      BT::InputPort<double>("rearm_clear_time", 1.5,
        "Seconds a sign must stay clear before the latch re-arms"),
      BT::InputPort<std::string>("trigger_classes", "stop sign,do not enter",
        "Comma-separated sign classes that trigger the stop"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void obstacleCallback(
    const tb4_perception_layer::msg::SemanticObstacleArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<msg::SemanticObstacleArray>::SharedPtr sub_;
  // Dedicated callback group + executor: the BT's shared node is not spun for
  // plugin subscriptions, so we service this subscription ourselves in tick().
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::mutex mutex_;
  // Timestamp of the most recent detection of a trigger sign within range.
  rclcpp::Time last_nearby_stamp_{0, 0, RCL_ROS_TIME};
  // Edge-triggered latch: fire SUCCESS once, then suppress until re-armed.
  bool armed_{true};
  double distance_threshold_{2.0};
  double rearm_clear_time_{1.5};
  std::set<std::string> trigger_classes_;
  bool initialized_{false};
};

}  // namespace tb4_perception_layer

#endif  // TB4_PERCEPTION_LAYER__IS_SIGN_NEARBY_HPP_
