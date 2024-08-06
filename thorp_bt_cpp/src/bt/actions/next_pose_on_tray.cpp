#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/TrayNextPose.h>

namespace thorp::bt::actions
{
/**
 * Calculate the next pose where to put an object on the tray.
 * Return FAILURE if the tray is full.
 */
class NextPoseOnTray : public BT::RosServiceNode<thorp_msgs::TrayNextPose, BT::SyncActionNode>
{
public:
  NextPoseOnTray(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("manipulation/tray/get_next_pose");
    ports.insert({ BT::OutputPort<geometry_msgs::PoseStamped>("pose_on_tray") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    setOutput("pose_on_tray", response.pose_on_tray);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(NextPoseOnTray);
};
}  // namespace thorp::bt::actions
