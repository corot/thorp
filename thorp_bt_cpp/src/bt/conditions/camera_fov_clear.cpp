#include <behaviortree_cpp/condition_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <std_srvs/srv/trigger.hpp>

namespace thorp::bt::condition
{
/**
 * Check if the fov of the manipulation camera is clear or it's blocked by a nearby object (possibly the arm).
 * Returns:
 * SUCCESS if the fov is clear
 * FAILURE if the fov is blocked
 */
class CameraFOVClear : public BT::RosServiceNode<std_srvs::srv::Trigger, BT::ConditionNode>
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
    RCLCPP_DEBUG_STREAM(logger(*this), response.message);
    return response.success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailedRequest(FailureCause failure) override
  {
    RCLCPP_ERROR_EXPRESSION(logger(*this), failure == MISSING_SERVER, "Check clear fov service not connected");
    RCLCPP_ERROR_EXPRESSION(logger(*this), failure == FAILED_CALL, "Check clear fov service call failed");
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(CameraFOVClear);
};
}  // namespace thorp::bt::condition
