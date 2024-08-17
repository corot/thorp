#include "thorp_bt_cpp/bt_ros_logger.hpp"

namespace BT
{

RosLogger::RosLogger(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode()), pnh_("~")
{
  bool expected = false;
  if (!ref_count_.compare_exchange_strong(expected, true))
  {
    throw LogicError("Only one instance of RosLogger shall be created");
  }
  pub_ = pnh_.advertise<std_msgs::String>("bt_status", 10);
}

RosLogger::~RosLogger()
{
  ref_count_.store(false);
}

void RosLogger::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status, NodeStatus status)
{
  //  using namespace std::chrono;
  //
  //  double since_epoch = duration<double>(timestamp).count();
  //  std_msgs::String msg;
  //  std::stringstream ss;
  //  ss << "[" << since_epoch << "]: " << node.name() << " " << toStr(prev_status, true) << " -> " << toStr(status,
  //  true); msg.data = ss.str();
  // Publish only actions the first tick they start running
  if (dynamic_cast<const BT::ActionNodeBase*>(&node) != nullptr && prev_status == NodeStatus::IDLE &&
      status == NodeStatus::RUNNING)
  {
    std_msgs::String msg;
    msg.data = node.name();
    pub_.publish(msg);
  }
}

void RosLogger::flush()
{
  ref_count_.store(false);
}

}  // namespace BT
