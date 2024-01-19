#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <std_srvs/Trigger.h>

namespace thorp::bt::actions
{
/**
 * Check if the gripper is physically holding an object, regardless of what the planning scene says
 * This should be a condition, but then we would need to handle the service instead of using RosServiceNode
 */
class ObjectAttached : public BT::RosServiceNode<std_srvs::Trigger>
{
public:
  ObjectAttached(const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode<ServiceType>(node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType>::providedPorts();
    ports["service_name"].setDefaultValue("obj_attached");
    ports.insert({ BT::OutputPort<std::string>("attached_object") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (response.success)
    {
      setOutput("attached_object", std::tolower(response.message, std::locale()));
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(ObjectAttached);
};
}  // namespace thorp::bt::actions
