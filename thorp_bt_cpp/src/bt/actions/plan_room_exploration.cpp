#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/srv/plan_room_exploration.hpp>

namespace thorp::bt::actions
{
/**
 * Plan the poses from where the camera sees all of a room, or of the whole map for room 0, with the exploration
 * planner, ordered to travel the least from the robot pose.
 */
class PlanRoomExploration : public BT::RosServiceNode<thorp_msgs::srv::PlanRoomExploration, BT::SyncActionNode>
{
public:
  PlanRoomExploration(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("exploration/plan_room_exploration");
    ports.insert({ BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),                    //
                   BT::InputPort<uint32_t>("room_number", "room to explore; 0 for the whole map"),  //
                   BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("coverage_waypoints") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.start = requireInput<geometry_msgs::msg::PoseStamped>(*this, "robot_pose");
    request.room = requireInput<uint32_t>(*this, "room_number");
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (!response.success)
    {
      RCLCPP_ERROR(logger(*this), "Room exploration planning failed: %s", response.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    setOutput("coverage_waypoints", response.poses);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(PlanRoomExploration);
};
}  // namespace thorp::bt::actions
