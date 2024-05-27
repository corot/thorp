#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <std_msgs/Empty.h>

namespace thorp::bt::actions
{
class EnableSafetyController : public BT::SyncActionNode
{
public:
  EnableSafetyController(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    pub_ = ros::NodeHandle().advertise<std_msgs::Empty>("kobuki_safety_controller/enable", 1);
  }

private:
  ros::Publisher pub_;

  BT::NodeStatus tick() override
  {
    pub_.publish(std_msgs::Empty());
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(EnableSafetyController);
};

class DisableSafetyController : public BT::SyncActionNode
{
public:
  DisableSafetyController(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    pub_ = ros::NodeHandle().advertise<std_msgs::Empty>("kobuki_safety_controller/disable", 1);
  }

private:
  ros::Publisher pub_;

  BT::NodeStatus tick() override
  {
    pub_.publish(std_msgs::Empty());
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(DisableSafetyController);
};
}  // namespace thorp::bt::actions
