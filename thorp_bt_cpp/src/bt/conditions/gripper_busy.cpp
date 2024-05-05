#include <behaviortree_cpp_v3/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <std_srvs/Trigger.h>

namespace thorp::bt::condition
{
/**
 * Check if we have a collision object attached to the gripper, according to the planning scene.
 */
class GripperBusy : public BT::RosServiceNode<std_srvs::Trigger, BT::ConditionNode>
{
public:
  GripperBusy(const std::string& name, const BT::NodeConfiguration& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("gripper_busy");
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
      ROS_INFO_STREAM(response.message);
      setOutput("attached_object", response.message);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(GripperBusy);
};
}  // namespace thorp::bt::condition
