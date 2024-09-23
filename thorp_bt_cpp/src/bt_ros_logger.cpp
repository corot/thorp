#include "thorp_bt_cpp/bt_ros_logger.hpp"

#include <thorp_msgs/BTNodeStatus.h>

namespace BT
{

RosLogger::RosLogger(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode()), pnh_("~")
{
  bool expected = false;
  if (!ref_count_.compare_exchange_strong(expected, true))
  {
    throw LogicError("Only one instance of RosLogger shall be created");
  }
  pub_ = pnh_.advertise<thorp_msgs::BTNodeStatus>("bt_status", 10);
}

RosLogger::~RosLogger()
{
  ref_count_.store(false);
}

void RosLogger::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status, NodeStatus status)
{
  thorp_msgs::BTNodeStatus msg;
  msg.stamp = ros::Time(0, timestamp.count());
  msg.name = node.name();
  msg.type = static_cast<uint8_t>(node.type());
  msg.status = static_cast<unsigned char>(status);
  msg.prev_status = static_cast<unsigned char>(prev_status);
  pub_.publish(msg);
}

void RosLogger::flush()
{
  ref_count_.store(false);
}

}  // namespace BT
