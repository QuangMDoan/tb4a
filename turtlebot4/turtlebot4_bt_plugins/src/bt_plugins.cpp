#include "behaviortree_cpp/bt_factory.h"

#include "turtlebot4_bt_plugins/is_goal_behind_condition.hpp"
#include "turtlebot4_bt_plugins/rotate_angle_action.hpp"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<turtlebot4_bt_plugins::IsGoalBehindCondition>("IsGoalBehind");
    factory.registerNodeType<turtlebot4_bt_plugins::RotateAngleAction>("RotateAngle");
}