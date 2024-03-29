#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/ros_subscriber_node.hpp"

#include <cob_perception_msgs/DetectionArray.h>

#include <thorp_toolkit/common.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Monitor tracked objects for a subset of objects.
 * @return BT::NodeStatus Returns SUCCESS if a tracked object is seen before timeout, FAILURE otherwise
 */
class MonitorObjects : public BT::RosSubscriberNode<cob_perception_msgs::DetectionArray>
{
public:
  MonitorObjects(const std::string& name, const BT::NodeConfiguration& conf) : RosSubscriberNode(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::RosSubscriberNode<cob_perception_msgs::DetectionArray>::providedPorts();
    ports["topic_name"].setDefaultValue("tracked_objects");
    ports.insert({ BT::InputPort<std::string>("target_objects", "Comma-separated list of objects to track"),
                   BT::OutputPort<cob_perception_msgs::Detection>("tracked_object"),
                   BT::OutputPort<geometry_msgs::PoseStamped>("tracked_object_pose") });
    return ports;
  }

private:
  void onStarted() override
  {
    auto target_objects_csv = getInput<std::string>("target_objects");
    if (target_objects_csv)
    {
      auto target_objects = ttk::tokenize(*target_objects_csv);
      std::copy(target_objects.begin(), target_objects.end(), inserter(valid_targets_, valid_targets_.begin()));
      ROS_INFO_STREAM_NAMED(name(), "Tracking " << *target_objects_csv);
    }
    else
    {
      ROS_INFO_STREAM_NAMED(name(), "No tracked objects provided; reporting all detections");
    }
  }

  inline BT::NodeStatus onReceived() override
  {
    cob_perception_msgs::DetectionArray msg = getMsgCopy();
    for (const auto& detection : msg.detections)
    {
      if (valid_targets_.count(detection.label))
      {
        ROS_INFO_STREAM_NAMED(name(), detection.label << " detected at " << ttk::toStr2D(detection.pose.pose.position));
        setOutput("tracked_object", detection);
        setOutput("tracked_object_pose", detection.pose);
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  std::unordered_set<std::string> valid_targets_;

  BT_REGISTER_NODE(MonitorObjects);
};
}  // namespace thorp::bt::actions
