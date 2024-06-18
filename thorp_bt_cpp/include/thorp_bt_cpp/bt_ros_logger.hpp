#ifndef BT_ROS_LOGGER_H
#define BT_ROS_LOGGER_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>

namespace BT
{
/**
 * @brief AddRosLoggerToTree. Given the root node of a tree,
 * a simple callback is subscribed to any status change of each node.
 *
 * @param root_node
 * @return Important: the returned shared_ptr must not go out of scope,
 *         otherwise the logger is removed.
 */

class RosLogger : public StatusChangeLogger
{
  inline static std::atomic<bool> ref_count_{false};
  ros::NodeHandle pnh_;
  ros::Publisher pub_;

public:
  RosLogger(const BT::Tree& tree);
  ~RosLogger() override;

  void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status, NodeStatus status) override;

  void flush() override;
};

}  // namespace BT

#endif  // BT_ROS_LOGGER_H
