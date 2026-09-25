#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <map>

#include <std_msgs/Empty.h>

namespace thorp::bt::actions
{
namespace
{
/**
 * A publisher for a one-shot command, outliving the tree that sends it.
 *
 * bt_server builds a tree per goal, so a publisher a node owns is advertised and destroyed
 * within one tick. Subscribers learn of a new topic asynchronously, so a command sent that soon
 * after advertising reaches nobody, and the safety controller is never actually switched. Only
 * the first use waits for someone to be listening.
 */
ros::Publisher& commandPub(const std::string& topic)
{
  static std::map<std::string, ros::Publisher> pubs;
  if (auto known = pubs.find(topic); known != pubs.end())
  {
    return known->second;
  }

  auto it = pubs.emplace(topic, ros::NodeHandle().advertise<std_msgs::Empty>(topic, 1)).first;
  for (int i = 0; i < 20 && it->second.getNumSubscribers() == 0; ++i)
  {
    ros::Duration(0.05).sleep();
  }
  return it->second;
}
}  // namespace

class EnableSafetyController : public BT::SyncActionNode
{
public:
  EnableSafetyController(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

private:
  BT::NodeStatus tick() override
  {
    commandPub("kobuki_safety_controller/enable").publish(std_msgs::Empty());
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(EnableSafetyController);
};

class DisableSafetyController : public BT::SyncActionNode
{
public:
  DisableSafetyController(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

private:
  BT::NodeStatus tick() override
  {
    commandPub("kobuki_safety_controller/disable").publish(std_msgs::Empty());
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(DisableSafetyController);
};
}  // namespace thorp::bt::actions
