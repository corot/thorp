#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <std_srvs/Trigger.h>

namespace thorp::bt::condition
{
/**
 * Check if the fov of the manipulation camera is clear or it's blocked by a nearby object (possibly the arm).
 * Returns:
 * SUCCESS if the fov is clear
 * FAILURE if the fov is blocked
 */
class CameraFOVClear : public BT::RosServiceNode<std_srvs::Trigger, BT::ConditionNode>
{
public:
  CameraFOVClear(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    // overwrite service_name with a default value
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("xtion_fov_analyzer/check_clear");
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    ROS_DEBUG_STREAM_NAMED(name(), response.message);
    return response.success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR_STREAM_COND_NAMED(failure == MISSING_SERVER, name(), "Check clear fov service not connected");
    ROS_ERROR_STREAM_COND_NAMED(failure == FAILED_CALL, name(), "Check clear fov service call failed");
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(CameraFOVClear);
};
}  // namespace thorp::bt::condition
