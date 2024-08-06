#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <std_srvs/Empty.h>

namespace thorp::bt::actions
{
class ClearRailMarkers : public BT::RosServiceNode<std_srvs::Empty, BT::SyncActionNode>
{
public:
  ClearRailMarkers(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("rail_segmentation/clear_markers");
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(ClearRailMarkers);
};
}  // namespace thorp::bt::actions
