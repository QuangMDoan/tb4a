#include "turtlebot4_bt_plugins/rotate_angle_action.hpp"

namespace turtlebot4_bt_plugins
{

RotateAngleAction::RotateAngleAction(
    const std::string & name, 
    const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<irobot_create_msgs::action::RotateAngle>(
    name, "rotate_angle", conf)
{
}

BT::PortsList RotateAngleAction::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("angle"),
        BT::InputPort<double>("max_rotation_speed", 1.0, "Max roation speed (rad/s)")
    });
}

void RotateAngleAction::on_tick()
{
    double angle = 0.0;
    double max_rotation_speed = 1.0;
    
    getInput("angle", angle);
    getInput("max_rotation_speed", max_rotation_speed);

    goal_.angle = angle;
    goal_.max_rotation_speed = max_rotation_speed;
}

} // namespace turtlebot4_bt_plugins