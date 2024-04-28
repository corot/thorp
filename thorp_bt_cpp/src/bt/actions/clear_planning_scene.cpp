#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/ClearPlanningScene.h>

namespace thorp::bt::actions
{
/**
 * Clear the planning scene, optionally sparing the tray and its content
 */
class ClearPlanningScene : public BT::RosServiceNode<thorp_msgs::ClearPlanningScene>
{
public:
  ClearPlanningScene(const std::string& name, const BT::NodeConfiguration& conf)
    : RosServiceNode<ServiceType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType>::providedPorts();
    ports["service_name"].setDefaultValue("manipulation/clear_planning_scene");
    ports.insert({ BT::InputPort<bool>("keep_tray") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.keep_tray = *getInput<bool>("keep_tray");
    ROS_INFO_NAMED(name(), "Clearing planning scene %s", request.keep_tray ? "but keeping tray and its content" : "");
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ClearPlanningScene);
};
}  // namespace thorp::bt::actions
