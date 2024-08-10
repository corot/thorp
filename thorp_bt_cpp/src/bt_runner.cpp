#include "thorp_bt_cpp/bt_runner.hpp"

// bt tools
#include <behaviortree_cpp/xml_parsing.h>

#include <thorp_toolkit/common.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt
{
BT::BehaviorTreeFactory Runner::bt_factory_{};

Runner::Runner() : pnh_("~")
{
}

bool Runner::loadTree()
{
  // on simulation, wait for the simulated time to start before loading and running the tree
  if (!ros::Time::waitForValid(ros::WallDuration(60)))
  {
    ROS_ERROR_STREAM_NAMED("bt_runner", "No valid time after 60 seconds");
    return false;
  }

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
    ROS_ERROR_STREAM_NAMED("bt_runner", "Unable to open file to write node models file: " << nodes_filepath);
    return false;
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
    return false;
  }

  // init publishers for debugging bt
  if (bt_ && pnh_.param<bool>("publish_bt", false))
  {
    bt_pub_groot_.emplace(*bt_);
    bt_pub_file_.emplace(*bt_, pnh_.param<std::string>("publish_bt_filepath", "/tmp/" + app_name_ + ".btlog").c_str());
    bt_pub_topic_.emplace(*bt_);
  }

  tick_rate_ = pnh_.param("tick_rate", 10.0);
  if (tick_rate_ <= 0)
  {
    ROS_WARN_NAMED("bt_runner", "Tick rate %.2f is non-positive, defaulting to 10 Hz", tick_rate_);
    tick_rate_ = 10;
  }
  return true;
}

void Runner::run()
{
  if (!bt_)
  {
    ROS_ERROR_STREAM_NAMED("bt_runner", "Behavior tree for " << app_name_ << " is not initialized");
    return;
  }

  ros::Duration(pnh_.param("start_delay", 0.0)).sleep();

  ros::Time time_start = ros::Time::now();

  using namespace std::chrono;
  auto tick_period = duration_cast<system_clock::duration>(duration<double>(1.0 / tick_rate_));
  auto status = BT::NodeStatus::RUNNING;
  while (ros::ok() && !BT::isStatusCompleted(status))
  {
    // Record the start time to subtract the time used on tick and ROS spin from the sleep time
    auto start_time = system_clock::now();

    status = bt_->tickOnce();
    ros::spinOnce();

    // Sleep tick period minus elapsed time
    auto elapsed_time = system_clock::now() - start_time;
    auto sleep_time = tick_period - elapsed_time;
    if (sleep_time > system_clock::duration::zero())
    {
      bt_->sleep(sleep_time);
    }
    else
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "bt_runner",
                              "Missed desired tick rate of %.2fHz, most recent tick actually took %.2f seconds",
                              tick_rate_, duration_cast<duration<double>>(elapsed_time).count());
    }
  }

  const double completion_time = (ros::Time::now() - time_start).toSec();
  ROS_INFO_NAMED("bt_runner", "%s completed in %.2fs with status %d", app_name_.c_str(), completion_time, (int)status);
}

}  // namespace thorp::bt
