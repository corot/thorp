#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/service_client_node.hpp"

#include <thorp_msgs/ConnectWaypoints.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class SmoothPath : public BT::RosServiceNode<thorp_msgs::ConnectWaypoints>
{
public:
  SmoothPath(const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode<ServiceType>(node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType>::providedPorts();
    ports["service_name"].setDefaultValue("waypoints_path/connect_waypoints");
    ports.insert({ BT::InputPort<std::vector<geometry_msgs::PoseStamped>>("waypoints"),
                   BT::OutputPort<nav_msgs::Path>("path") });
    return ports;
  }

  void sendRequest(RequestType& request) override
  {
    request.max_steps = 5;
    request.max_radius = 0.5;
    request.resolution = 0.15;
    request.visualize_path = true;
    getInput("waypoints", request.waypoints);
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    setOutput<nav_msgs::Path>("path", response.path);
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("Smooth path failed %d", static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  BT_REGISTER_NODE(SmoothPath);
};
}  // namespace thorp::bt::actions
