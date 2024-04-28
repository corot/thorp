#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/TrayCapacity.h>

namespace thorp::bt::conditions
{
/**
 * Check whether the tray is full.
 * Return SUCCESS if the tray is full.
 */
class TrayFull : public BT::RosServiceNode<thorp_msgs::TrayCapacity>
{
public:
  TrayFull(const std::string& name, const BT::NodeConfiguration& conf) : RosServiceNode<ServiceType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType>::providedPorts();
    ports["service_name"].setDefaultValue("manipulation/tray/get_capacity");
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    bool tray_full = response.current == 0;
    ROS_DEBUG_NAMED(name(), "The tray is full");
    return tray_full ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(TrayFull);
};
}  // namespace thorp::bt::conditions
