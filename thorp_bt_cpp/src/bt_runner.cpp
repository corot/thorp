#include "thorp_bt_cpp/bt_runner.hpp"

#include "thorp_bt_cpp/bt_nodes.hpp"

namespace thorp::bt
{
Runner::Runner() : pnh_("~")
{
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
    bt_pub_file_.emplace(*bt_, pnh_.param<std::string>("publish_bt_filepath", "pub_bt.xml").c_str());
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
  /*if (!bt_)
  {
    ROS_ERROR_STREAM_NAMED("bt_runner", "Behavior tree is not initialized");
    result.status = mbf_msgs::ExePathResult::INTERNAL_ERROR;
    result.remarks = mbf_utility::outcome2str(result.status);
    navigate_as_.setAborted(result);
    last_navigate_action_result_ = result;
    return;
  }
*/
  ros::Rate rate(tick_rate_);
  auto status = BT::NodeStatus::RUNNING;
  while (ros::ok() && status == BT::NodeStatus::RUNNING)
  {
    status = bt_->tickRoot();

    if (!rate.sleep())
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "bt_runner",
                              "Missed desired tick rate of %.2fHz, most recent tick actually took %.2f seconds",
                              tick_rate_, rate.cycleTime().toSec());
    }
  }

/*
  // fill details related to robot pose
  if (robot_info_.getRobotPose(result.final_pose) && !goal->goals.empty())
  {
    result.angle_to_goal = rrg::angle(result.final_pose, goal->goals.front().pose);
    result.dist_to_goal = rrg::distance(result.final_pose, goal->goals.front().pose);
  }

  if (status == BT::NodeStatus::FAILURE)
  {
    // default to internal error
    result.status = mbf_msgs::ExePathResult::INTERNAL_ERROR;

    try
    {
      bt_->rootBlackboard()->get("error", result.status);
    }
    catch (const std::exception& e)
    {
      // do nothing since we already set default value above
    }

    result.remarks = mbf_utility::outcome2str(result.status);

    ROS_ERROR_NAMED("bt_runner", "Navigation failed at %.2f, %.2f, %.2f; distance to goal: %.2f, angle to goal: %.2f",
                    result.final_pose.pose.position.x, result.final_pose.pose.position.y,
                    rrg::getYaw(result.final_pose), result.dist_to_goal, result.angle_to_goal);
    ROS_ERROR_NAMED("bt_runner", "Error %d: %s", result.status, mbf_utility::outcome2str(result.status).c_str());

    navigate_as_.setAborted(result);
    last_navigate_action_result_ = result;
    return;
  }

  ROS_INFO_NAMED("bt_runner", "Navigation finished at %.2f, %.2f, %.2f; distance to goal: %.2f, angle to goal: %.2f",
                 result.final_pose.pose.position.x, result.final_pose.pose.position.y, rrg::getYaw(result.final_pose),
                 result.dist_to_goal, result.angle_to_goal);

  result.status = mbf_msgs::ExePathResult::SUCCESS;
  result.remarks = mbf_utility::outcome2str(result.status);
  last_navigate_action_result_ = result;
  navigate_as_.setSucceeded(result);
*/
}

}  // namespace thorp::bt
