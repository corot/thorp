#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/srv/plan_room_sequence.hpp>

namespace thorp::bt::actions
{
/**
 * Order the rooms to visit them all from the robot pose, traveling the least, with the exploration planner. Rooms
 * the robot can't reach are left out.
 */
class PlanRoomSequence : public BT::RosServiceNode<thorp_msgs::srv::PlanRoomSequence, BT::SyncActionNode>
{
public:
  PlanRoomSequence(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("exploration/plan_room_sequence");
    ports.insert({ BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),  //
                   BT::InputPort<std::vector<uint32_t>>("rooms"),                 //
                   BT::OutputPort<std::vector<uint32_t>>("room_sequence") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.start = requireInput<geometry_msgs::msg::PoseStamped>(*this, "robot_pose");
    request.rooms = requireInput<std::vector<uint32_t>>(*this, "rooms");
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (!response.success)
    {
      RCLCPP_ERROR(logger(*this), "Room sequence planning failed: %s", response.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    setOutput("room_sequence", response.sequence);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(PlanRoomSequence);
};
}  // namespace thorp::bt::actions
