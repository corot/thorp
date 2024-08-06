#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/TrayAddObject.h>

namespace thorp::bt::actions
{
/**
 * Add a collision object to the tray.
 */
class AddObjectToTray : public BT::RosServiceNode<thorp_msgs::TrayAddObject, BT::SyncActionNode>
{
public:
  AddObjectToTray(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("manipulation/tray/add_object");
    ports.insert({ BT::InputPort<std::string>("object_name"),  //
                   BT::InputPort<geometry_msgs::PoseStamped>("pose_on_tray") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.object_name = *getInput<std::string>("object_name");
    request.pose_on_tray = *getInput<geometry_msgs::PoseStamped>("pose_on_tray");
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(AddObjectToTray);
};
}  // namespace thorp::bt::actions
