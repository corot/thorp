#include "thorp_bt_cpp/bt_runner.hpp"

#include "thorp_bt_cpp/bt_nodes.hpp"

#include <thorp_toolkit/common.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt
{
Runner::Runner() : pnh_("~")
{
  // on simulation, wait for the simulated time to start before loading and running the tree
  ttk::waitForSimTime();

  const std::string app_name = pnh_.param<std::string>("app_name", "");
  const std::string bt_filepath = pnh_.param<std::string>("bt_filepath", "bt.xml");
  const std::string nodes_filepath = pnh_.param<std::string>("nodes_filepath", "nodes.xml");

  // register behavior tree nodes
  registerNodes(bt_factory_, pnh_, nodes_filepath);

  try
  {
    bt_factory_.registerBehaviorTreeFromFile(bt_filepath);
    bt_ = bt_factory_.createTree(app_name);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_NAMED("bt_runner", "Failed to load behavior tree " << bt_filepath);
    ROS_ERROR_STREAM_NAMED("bt_runner", e.what());
    bt_.reset();
  }

  // init publishers for debugging bt
  if (bt_ && pnh_.param<bool>("publish_bt", false))
  {
    bt_pub_zmq_.emplace(*bt_);
    bt_pub_file_.emplace(*bt_, pnh_.param<std::string>("publish_bt_filepath", "/tmp/pub_bt.xml").c_str());
  }

  tick_rate_ = pnh_.param("tick_rate", 10.0);
  if (tick_rate_ <= 0)
  {
    ROS_WARN_NAMED("bt_runner", "Tick rate %.2f is non-positive, defaulting to 10 Hz", tick_rate_);
    tick_rate_ = 10;
  }
}

void Runner::run()
{
  if (!bt_)
  {
    ROS_ERROR_STREAM_NAMED("bt_runner", "Behavior tree is not initialized");
    return;
  }

  ros::Rate rate(tick_rate_);
  auto status = BT::NodeStatus::RUNNING;
  while (ros::ok() && status == BT::NodeStatus::RUNNING)
  {
    status = bt_->tickRoot();

    ros::spinOnce();

    if (!rate.sleep())
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "bt_runner",
                              "Missed desired tick rate of %.2fHz, most recent tick actually took %.2f seconds",
                              tick_rate_, rate.cycleTime().toSec());
    }
  }
}

}  // namespace thorp::bt
