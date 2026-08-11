#ifndef TURTLEBOT4_BT_PLUGINS__IS_GOAL_BEHIND_CONDITION_HPP_
#define TURTLEBOT4_BT_PLUGINS__IS_GOAL_BEHIND_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace turtlebot4_bt_plugins
{

class IsGoalBehindCondition : public BT::ConditionNode 
{
public:
    IsGoalBehindCondition(const std::string & name, const BT::NodeConfiguration & conf);
    
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace turtlebot4_bt_plugins

#endif  // TURTLEBOT4_BT_PLUGINS__IS_GOAL_BEHIND_CONDITION_HPP_