#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/actions/subscriber_node.hpp"

#include <cob_perception_msgs/DetectionArray.h>

#include <thorp_toolkit/common.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Monitor tracked objects for a subset of objects.
 * @return BT::NodeStatus Returns SUCCESS if a tracked object is seen before timeout, FAILURE otherwise
 */
class MonitorObjects : public BT::SubscriberNode<cob_perception_msgs::DetectionArray>
{
public:
  MonitorObjects(const std::string& name, const BT::NodeConfiguration& conf) : SubscriberNode(name, conf)
  {
    ROS_ERROR_STREAM("c");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("target_objects", "Comma-separated list of objects to track"),
             BT::OutputPort<cob_perception_msgs::Detection>("tracked_object"),
             BT::OutputPort<geometry_msgs::PoseStamped>("tracked_object_pose") };
  }

protected:
  void onStarted() override
  {
    ROS_ERROR_STREAM("s");
    auto target_objects_csv = getInput<std::string>("target_objects");
    ROS_ERROR_STREAM_NAMED(name(), " \n\nSTARTED " <<target_objects_csv.has_value());
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
    ROS_ERROR_STREAM_NAMED(name(), "on rec");
    cob_perception_msgs::DetectionArray msg = getMsgCopy();
    ROS_ERROR_STREAM_NAMED(name(), msg.detections.size() << " detected     <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   ");
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

private:
  std::unordered_set<std::string> valid_targets_;
};

}  // namespace thorp::bt::actions
