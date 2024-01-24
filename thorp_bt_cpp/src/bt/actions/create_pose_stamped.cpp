#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class CreatePoseStamped : public BT::SyncActionNode
{
public:
  CreatePoseStamped(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("x"),
             BT::InputPort<double>("y"),
             BT::InputPort<double>("yaw"),
             BT::InputPort<std::string>("frame"),
             BT::OutputPort<geometry_msgs::PoseStamped>("pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    setOutput("pose", ttk::createPose(*getInput<double>("x"),
                                      *getInput<double>("y"),
                                      *getInput<double>("yaw"),
                                      *getInput<std::string>("frame")));
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(CreatePoseStamped);
};
}  // namespace thorp::bt::actions
