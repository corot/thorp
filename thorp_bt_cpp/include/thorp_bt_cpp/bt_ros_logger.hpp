#pragma once

#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/loggers/abstract_logger.h>

#include <thorp_msgs/msg/bt_node_status.hpp>

namespace BT
{
/**
 * @brief Publish every node status change of a tree as a thorp_msgs/BTNodeStatus message, on ~/bt_status.
 * Only one instance can exist at a time.
 */
class RosLogger : public StatusChangeLogger
{
  inline static std::atomic<bool> ref_count_{ false };
  rclcpp::Publisher<thorp_msgs::msg::BTNodeStatus>::SharedPtr pub_;

public:
  RosLogger(const BT::Tree& tree, const rclcpp::Node::SharedPtr& node);
  ~RosLogger() override;

  void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status, NodeStatus status) override;

  void flush() override;
};

}  // namespace BT
