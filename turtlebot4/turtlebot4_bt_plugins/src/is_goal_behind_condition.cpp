#include "turtlebot4_bt_plugins/is_goal_behind_condition.hpp"

#include <cmath>
#include <stdexcept>

#include "behaviortree_cpp/bt_factory.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace turtlebot4_bt_plugins
{

IsGoalBehindCondition::IsGoalBehindCondition(
    const std::string & name, 
    const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
    try {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    } catch (const std::exception & ex) {
        throw BT::RuntimeError(std::string("IsGoalBehindCondition missing BT blackboard entries: ") + ex.what());
    }

    if(!node_ || !tf_buffer_){
        throw BT::RuntimeError("IsGoalBehindCondition requires 'node' and 'tf_buffer' on the BT blackboard");
    }
}

BT::PortsList IsGoalBehindCondition::providedPorts()
{
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame"),
        BT::InputPort<double>("behind_angle", 1.57, "Angle threshold in radians"),
        BT::InputPort<double>("min_distance", 0.2, "Minimum distance to consider goal"),
        BT::OutputPort<double>("angle")
    };
}

BT::NodeStatus IsGoalBehindCondition::tick()
{
    geometry_msgs::msg::PoseStamped goal;
    if(!getInput("goal", goal)) {
        RCLCPP_WARN(node_->get_logger(), "IsGoalBehind: missing input [goal]");
        return BT::NodeStatus::FAILURE;
    }

    std::string base_frame = "base_link";
    getInput("robot_base_frame", base_frame);

    double behind_angle = 1.57;
    getInput("behind_angle", behind_angle);

    double min_distance = 0.2;
    getInput("min_distance", min_distance);

    geometry_msgs::msg::PoseStamped goal_base;
    try {
        auto transform = tf_buffer_->lookupTransform(
            base_frame, goal.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(goal, goal_base, transform);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "IsGoalBehind: transform failed: %s", ex.what());
        return BT::NodeStatus::FAILURE;
    }

    const double dx = goal_base.pose.position.x;
    const double dy = goal_base.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance < min_distance) {
        return BT::NodeStatus::FAILURE;
    }

    const double angle = std::atan2(dy, dx);
    setOutput("angle", angle);

    if (std::abs(angle) > behind_angle) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace turtlebot4_bt_plugins