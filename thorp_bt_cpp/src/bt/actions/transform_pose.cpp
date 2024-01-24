#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class TransformPose : public BT::SyncActionNode
{
public:
  TransformPose(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("x"),      //
             BT::InputPort<double>("y"),      //
             BT::InputPort<double>("z"),      //
             BT::InputPort<double>("roll"),   //
             BT::InputPort<double>("pitch"),  //
             BT::InputPort<double>("yaw"),    //
             BT::BidirectionalPort<geometry_msgs::PoseStamped>("pose") };
  }

private:
  BT::NodeStatus tick() override
  {
    auto o_x = getInput<double>("x");
    auto o_y = getInput<double>("y");
    auto o_z = getInput<double>("z");
    auto o_roll = getInput<double>("roll");
    auto o_pitch = getInput<double>("pitch");
    auto o_yaw = getInput<double>("yaw");
    double x = o_x ? *o_x : 0.0;
    double y = o_y ? *o_y : 0.0;
    double z = o_z ? *o_z : 0.0;
    double roll = o_roll ? *o_roll : 0.0;
    double pitch = o_pitch ? *o_pitch : 0.0;
    double yaw = o_yaw ? *o_yaw : 0.0;
    auto pose = *getInput<geometry_msgs::PoseStamped>("pose");
    geometry_msgs::TransformStamped tf =
        ttk::toTransform(ttk::createPose(x, y, z, roll, pitch, yaw, pose.header.frame_id));
    tf2::doTransform(pose, pose, tf);
    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(TransformPose);
};
}  // namespace thorp::bt::actions
