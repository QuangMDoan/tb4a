#ifndef TURTLEBOT4_BT_PLUGINS__ROTATE_ANGLE_ACTION_HPP_
#define TURTLEBOT4_BT_PLUGINS__ROTATE_ANGLE_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"

namespace turtlebot4_bt_plugins
{

class RotateAngleAction 
    : public nav2_behavior_tree::BtActionNode<irobot_create_msgs::action::RotateAngle>
{
public:
    RotateAngleAction(const std::string & name, const BT::NodeConfiguration & conf);

    static BT::PortsList providedPorts();
    
    void on_tick() override;
};

}  // namespace turtlebot4_bt_plugins

#endif  // TURTLEBOT4_BT_PLUGINS__ROTATE_ANGLE_ACTION_HPP_