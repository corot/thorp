#include "thorp_bt_cpp/bt_runner.hpp"

// bt tools
#include <behaviortree_cpp_v3/xml_parsing.h>

#include <thorp_toolkit/common.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt
{
BT::BehaviorTreeFactory Runner::bt_factory_{};

Runner::Runner() : pnh_("~")
{
  // on simulation, wait for the simulated time to start before loading and running the tree
  ttk::waitForSimTime();

  std::string bt_filepath, nodes_filepath;
  pnh_.getParam("app_name", app_name_);
  pnh_.getParam("bt_filepath", bt_filepath);
  pnh_.getParam("nodes_filepath", nodes_filepath);

  // dump to an XML file to load on Groot
  std::ofstream ofs;
  ofs.open(nodes_filepath,
           std::ios::out |        // output file stream
               std::ios::trunc);  // truncate content
  if (ofs)
  {
    ofs << BT::writeTreeNodesModelXML(bt_factory_) << std::endl;
    ofs.close();
  }
  else
  {
    ROS_ERROR_STREAM("Unable to open file to write node models file: " << nodes_filepath);
  }

  try
  {
    bt_factory_.registerBehaviorTreeFromFile(bt_filepath);
    bt_ = bt_factory_.createTree(app_name_);
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
    ROS_ERROR_STREAM_NAMED("bt_runner", "Behavior tree for " << app_name_ << " is not initialized");
    return;
  }

  ros::Time time_start = ros::Time::now();
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
  const double completion_time = (ros::Time::now() - time_start).toSec();
  ROS_INFO_NAMED("bt_runner", "%s completed in %.2fs with status %d", app_name_.c_str(), completion_time, (int)status);
}

}  // namespace thorp::bt
