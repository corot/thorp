#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_service_node.hpp"

#include <rail_manipulation_msgs/SegmentObjects.h>

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Segment the observed scene in search of tables
 */
class DetectTables : public BT::RosServiceNode<rail_manipulation_msgs::SegmentObjects>
{
public:
  DetectTables(const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode<ServiceType>(node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosServiceNode<ServiceType>::providedPorts();
    ports["service_name"].setDefaultValue("rail_segmentation/segment_objects");
    ports.insert({ BT::OutputPort<rail_manipulation_msgs::SegmentedObject>("table"),  //
                   BT::OutputPort<geometry_msgs::PoseStamped>("table_pose") });
    return ports;
  }

private:
  void sendRequest(RequestType& request) override
  {
    request.only_surface = true;
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    if (response.segmented_objects.objects.empty())
    {
      // No tables detected
      return BT::NodeStatus::FAILURE;
    }
    auto table = response.segmented_objects.objects.front();
    table.name = "table";
    geometry_msgs::PoseStamped table_pose;
    table_pose.header = table.point_cloud.header;
    table_pose.pose.position = table.center;
    table_pose.pose.orientation = table.orientation;
    if (!ttk::TF2::instance().transformPose("map", table_pose, table_pose))
    {
      // Transform table pose to map frame failed
      return BT::NodeStatus::FAILURE;
    }
    double width = table.width, length = table.depth;
    ROS_INFO_NAMED(name(), "Detected table of size %.1f x %.1f at %s", width, length, ttk::toCStr2D(table_pose));
    setOutput("table", table);
    setOutput("table_pose", table_pose);

    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR_NAMED(name(), "Segment objects failed %d", static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

  BT_REGISTER_NODE(DetectTables);
};
}  // namespace thorp::bt::actions
