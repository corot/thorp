#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <std_srvs/Empty.h>

namespace thorp::bt::actions
{
class ClearCostmaps : public BT::RosServiceNode<std_srvs::Empty>
{
public:
  ClearCostmaps(const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode<ServiceType>(node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType>::providedPorts();
    ports["service_name"].setDefaultValue("move_base_flex/clear_costmaps");
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

  BT_REGISTER_NODE(ClearCostmaps);
};
}  // namespace thorp::bt::actions
