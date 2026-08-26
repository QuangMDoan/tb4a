#ifndef TURTLEBOT4_BT_PLUGINS__RANDOM_SELECTOR_HPP_
#define TURTLEBOT4_BT_PLUGINS__RANDOM_SELECTOR_HPP_

#include <random>
#include <string>

#include "behaviortree_cpp/control_node.h"

namespace turtlebot4_bt_plugins
{

// Control node that ticks ONE uniformly-random child per entry (re-randomised
// each time it is re-entered), instead of the fixed order of Sequence/RoundRobin.
// Used so recovery picks Spin or BackUp at random.
class RandomSelector : public BT::ControlNode
{
public:
  RandomSelector(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ControlNode(name, conf),
    gen_(std::random_device{}())
  {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    const size_t n = childrenCount();
    if (n == 0) {return BT::NodeStatus::FAILURE;}

    if (selected_ < 0) {
      std::uniform_int_distribution<int> dist(0, static_cast<int>(n) - 1);
      selected_ = dist(gen_);
    }

    setStatus(BT::NodeStatus::RUNNING);
    const BT::NodeStatus child_status = children_nodes_[selected_]->executeTick();

    if (child_status != BT::NodeStatus::RUNNING) {
      haltChildren();
      selected_ = -1;  // re-randomise on next entry
    }
    return child_status;
  }

  void halt() override
  {
    selected_ = -1;
    BT::ControlNode::halt();
  }

private:
  std::mt19937 gen_;
  int selected_{-1};
};

}  // namespace turtlebot4_bt_plugins

#endif  // TURTLEBOT4_BT_PLUGINS__RANDOM_SELECTOR_HPP_
