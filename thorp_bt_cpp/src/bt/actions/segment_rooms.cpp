#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <thorp_msgs/srv/segment_rooms.hpp>

namespace thorp::bt::actions
{
/**
 * Segment the map into rooms, with the exploration planner, that keeps them for planning their sequence and
 * exploration. Provides the rooms' ids.
 */
class SegmentRooms : public BT::RosServiceNode<thorp_msgs::srv::SegmentRooms, BT::SyncActionNode>
{
public:
  SegmentRooms(const std::string& name, const BT::NodeConfig& conf)
    : RosServiceNode<ServiceType, ParentType>(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType, ParentType>::providedPorts();
    ports["service_name"].setDefaultValue("exploration/segment_rooms");
    ports.insert(BT::OutputPort<std::vector<uint32_t>>("rooms"));
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (!response.success)
    {
      RCLCPP_ERROR(logger(*this), "Room segmentation failed: %s", response.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    std::vector<uint32_t> rooms;
    for (const auto& room : response.rooms)
      rooms.push_back(room.id);
    RCLCPP_INFO(logger(*this), "Map segmented into %zu rooms", rooms.size());
    setOutput("rooms", rooms);
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(SegmentRooms);
};
}  // namespace thorp::bt::actions
