#include "thorp_bt_cpp/bt_ros_logger.hpp"

namespace BT
{
RosLogger::RosLogger(const BT::Tree& tree, const rclcpp::Node::SharedPtr& node) : StatusChangeLogger(tree.rootNode())
{
  bool expected = false;
  if (!ref_count_.compare_exchange_strong(expected, true))
  {
    throw LogicError("Only one instance of RosLogger shall be created");
  }
  pub_ = node->create_publisher<thorp_msgs::msg::BTNodeStatus>("~/bt_status", 10);
}

RosLogger::~RosLogger()
{
  ref_count_.store(false);
}

void RosLogger::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status, NodeStatus status)
{
  thorp_msgs::msg::BTNodeStatus msg;
  msg.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp).count());
  msg.name = node.name();
  msg.type = static_cast<uint8_t>(node.type());
  msg.status = static_cast<unsigned char>(status);
  msg.prev_status = static_cast<unsigned char>(prev_status);
  pub_->publish(msg);
}

void RosLogger::flush()
{
  ref_count_.store(false);
}

}  // namespace BT
