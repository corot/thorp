#include <future>
#include <random>

#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Path.h>
#include <mbf_msgs/RecoveryActionResult.h>
#include <ros/common.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_mbf_plugins/slow_escape_recovery.h"

PLUGINLIB_EXPORT_CLASS(thorp_mbf_plugins::SlowEscapeRecovery, mbf_costmap_core::CostmapRecovery)

namespace thorp_mbf_plugins
{


SlowEscapeRecovery::SlowEscapeRecovery()
                  : initialized_(false), global_costmap_(nullptr), local_costmap_(nullptr),
                    behavior_running_(false), cancel_requested_(false)
{
}

SlowEscapeRecovery::~SlowEscapeRecovery()
{
}

void SlowEscapeRecovery::initialize(std::string n, tf2_ros::Buffer* tf,
                                    costmap_2d::Costmap2DROS* global_costmap,
                                    costmap_2d::Costmap2DROS* local_costmap)
{
  name_ = n;

  local_costmap_ = local_costmap;
  global_costmap_ = global_costmap;

  ros::NodeHandle private_nh("~/" + n);

  // Load behavior parameters
  private_nh.param("v_max", v_max_, 0.1);
  private_nh.param("w_max", w_max_, 0.2);
  private_nh.param("wheel_separation", wheel_separation_, 0.23);  // kobuki's base
  private_nh.param("controller_frequency", controller_frequency_, 10.0);
  private_nh.param("vel_projection_time", vel_projection_time_, 0.5);
  private_nh.param("stagnated_patience", stagnated_patience_, 1.0);  // unused by now XXX
  private_nh.param("max_execution_time", max_execution_time_, 10.0);
  private_nh.param("max_travelled_distance", max_travelled_distance_, 0.2);
  private_nh.param("escape_from_collision", escape_from_collision_, -1.0);  // disabled by default
  private_nh.param("escape_from_inscribed", escape_from_inscribed_, -1.0);  // disabled by default
  private_nh.param("release_bumper_distance", release_bumper_distance_, 0.04);
  private_nh.param("max_search_depth", max_search_depth_, 30);
  if (escape_from_inscribed_ >= 0.0 && escape_from_collision_ >= 0.0)
  {
    ROS_WARN("escape_to_free_space is not negative (%g), so it overrules escape_from_collision (%g)",
             escape_from_inscribed_, escape_from_collision_);
    escape_from_collision_ = -1.0;
  }

  release_bumper_depth_ = (int)std::round(release_bumper_distance_ / (v_max_ / controller_frequency_));

  ros::NodeHandle nh;

  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  path_pub_ = private_nh.advertise<nav_msgs::Path>("escape_plan", 1);

  initialized_ = true;
}

uint32_t SlowEscapeRecovery::runBehavior(std::string& message)
{
  if (!initialized_)
  {
    ROS_ERROR("[%s] recovery behavior has not been initialized, doing nothing", name_.c_str());
    return mbf_msgs::RecoveryResult::NOT_INITIALIZED;
  }

  ROS_WARN("[%s] recovery behavior started", name_.c_str());

  behavior_running_ = true;

  // save current position and time to track elapsed time and traveled distance
  ros::Time start_time = ros::Time::now();
  geometry_msgs::PoseStamped local_robot_pose, global_robot_pose, initial_global_robot_pose;
  local_costmap_->getRobotPose(local_robot_pose);
  global_costmap_->getRobotPose(global_robot_pose);
  initial_global_robot_pose = global_robot_pose;

  std::vector<geometry_msgs::Point> local_costmap_footprint = local_costmap_->getUnpaddedRobotFootprint();
  std::vector<geometry_msgs::Point> global_costmap_footprint = global_costmap_->getUnpaddedRobotFootprint();
  if (escape_from_collision_ >= 0.0)
  {
    // pad local and global costmap footprints with the distance we consider as away from collision
    costmap_2d::padFootprint(local_costmap_footprint, escape_from_collision_);
    costmap_2d::padFootprint(global_costmap_footprint, escape_from_collision_);
  }
  if (escape_from_inscribed_ >= 0.0)
  {
    // pad local and global costmap footprints with the distance we consider as away from inscribed lethal obstacles
    costmap_2d::padFootprint(local_costmap_footprint, escape_from_inscribed_);
    costmap_2d::padFootprint(global_costmap_footprint, escape_from_inscribed_);
  }

  ros::Rate rate(controller_frequency_);
  double planning_time_percent = escape_from_collision_ >= 0.0 ? 75.0 : 95.0;
  std::chrono::milliseconds planning_time(int64_t((planning_time_percent * 10.0)/controller_frequency_));

  unsigned int stagnated = 0;
  double cost = std::numeric_limits<double>::infinity();
  double new_cost = std::numeric_limits<double>::infinity();
  double travelled_distance = 0.0, elapsed_time = 0.0;

  std::future<PlanResult> future;
  std::vector<geometry_msgs::Twist> cmd_vels, new_cmd_vels;

  // Run until...
  while (ros::ok())
  {
    if (cancel_requested_)
    {
      // ...we get canceled
      ROS_INFO("[%s] recovery behavior canceled", name_.c_str());
      break;
    }

    if (escape_from_inscribed_ >= 0.0 || escape_from_collision_ >= 0.0)
    {
      // ...controller runs until robot has escaped from lethal/inflated obstacles by a given distance

      bool escaped = true;
      double local_pose_cost = getPoseCost(local_robot_pose.pose.position.x,
                                           local_robot_pose.pose.position.y,
                                           tf2::getYaw(local_robot_pose.pose.orientation),
                                           local_costmap_, local_costmap_footprint, escaped);
      if (escaped)
      {
        double global_pose_cost = getPoseCost(global_robot_pose.pose.position.x,
                                              global_robot_pose.pose.position.y,
                                              tf2::getYaw(global_robot_pose.pose.orientation),
                                              global_costmap_, global_costmap_footprint, escaped);
        if (escaped)
        {
          ROS_INFO("[%s] escaped from %s obstacle (current costs: %d, %d)",
                   name_.c_str(), escape_from_inscribed_ >= 0.0 ? "inscribed" : "lethal",
                   (int)local_pose_cost, (int)global_pose_cost);
          break;
        }
      }
    }

    // Start planning if not already running and wait for completion 95% of our controlling period (we keep the
    // remaining 5% for analyzing the outcome and updating robot pose, more than enough). When moving away from
    // collision, we wait 75% of the time instead, as checking current pose on global costmap can take more time
    // vel_projection_time_ value is critical, as it makes pose change enough so cost differences are meaningful
    // increase by 20% when we get ZERO_VEL or INC_COST to increase the chances of getting a working scape plan
    if (!future.valid())
      future = std::async(std::launch::async, &SlowEscapeRecovery::makePlan, this,
                          std::cref(local_robot_pose), vel_projection_time_ * (1 + stagnated * 0.2), cost,
                          std::ref(new_cost), std::ref(new_cmd_vels));

    PlanResult result = future.wait_for(planning_time) == std::future_status::ready ? future.get()
                                                                                    : PlanResult::PLANNING;
    if (result != PlanResult::PLANNING)
    {
      // Planning completed; analyze the outcome
      if ((result == PlanResult::INC_COST || result == PlanResult::ZERO_VEL) && cmd_vels.empty())
        stagnated++;
      else
        stagnated = 0;

      if (stagnated > 5)
      {
        // Looks like we are going nowhere: planner cannot decrease current cost, or produces zero velocities
        ROS_INFO("[%s] robot stagnated; assume local minimum and stop", name_.c_str());
        break;
      }

      // update our plan if he brings us to lower cost or if we don't have any plan at all
      if (cmd_vels.empty() || new_cost < cost)
      {
        cmd_vels = new_cmd_vels;
        cost = new_cost;
      }
    }

    travelled_distance = ttk::distance2D(initial_global_robot_pose, global_robot_pose);
    if (travelled_distance > max_travelled_distance_)
    {
      // ...controller moves for more than max_travelled_distance_
      ROS_INFO("[%s] maximum distance reached (%gm)", name_.c_str(), max_travelled_distance_);
      break;
    }

    elapsed_time = (ros::Time::now() - start_time).toSec();
    if (elapsed_time > max_execution_time_)
    {
      // ...controller runs for up to max_execution_time_ seconds
      ROS_INFO("[%s] maximum allowed time reached (%gs)", name_.c_str(), max_execution_time_);
      break;
    }

    if (!cmd_vels.empty())
    {
      cmd_vel_pub_.publish(cmd_vels.front());
      cmd_vels.erase(cmd_vels.begin());
    }
    else if (result == PlanResult::ZERO_COST)
    {
      // pretty sure we will never reach this wonderland case...
      ROS_INFO("[%s] Reached zero-cost, so stopping", name_.c_str());
      break;
    }

    if (!rate.sleep())
    {
      ROS_WARN_THROTTLE(1.0, "[%s] Controller period not met (expected %gs, real %gs)",
                        name_.c_str(), rate.expectedCycleTime().toSec(), rate.cycleTime().toSec());
    }

    local_costmap_->getRobotPose(local_robot_pose);
    global_costmap_->getRobotPose(global_robot_pose);
  }
  ROS_INFO("[%s] recovery behavior finished; moved %.2fm in %.2fs", name_.c_str(), travelled_distance, elapsed_time);

  if (future.valid())
  {
    // If planning task is still running, stop it and wait until it returns (should be very fast)
    cancel_requested_ = true;
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready)
      ROS_WARN("[%s] Planning thread not responding for 1 second", name_.c_str());
  }

  cancel_requested_ = false;
  behavior_running_ = false;

  // TODO: shouldn't I fail in case of not moving?   OJO!!! can also rotate in place!!! need to check angle   ->> maybe count the nb of commands and fail if 0

  return mbf_msgs::RecoveryResult::SUCCESS;
}

SlowEscapeRecovery::PlanResult SlowEscapeRecovery::makePlan(const geometry_msgs::PoseStamped& robot_pose,
                                                            double time_step, double prev_cost, double& new_cost,
                                                            std::vector<geometry_msgs::Twist>& cmd_vels)
{
  ros::Time t0 = ros::Time::now();

  std::priority_queue<CmdVelNode*, std::vector<CmdVelNode*>, CmpCmdVelNodePtrs> open_nodes;
  geometry_msgs::Twist cmd_vel;
  CmdVelNode* current_node = new CmdVelNode(0, robot_pose.pose.position.x, robot_pose.pose.position.y,
                                            tf2::getYaw(robot_pose.pose.orientation), cmd_vel);
  CmdVelNode* root_node = current_node;
  open_nodes.push(current_node);
  CmdVelNode* best_node = current_node;

  bool escaped = true;
  double initial_cost = getPoseCost(current_node->pose_x_, current_node->pose_y_, current_node->heading_,
                                    local_costmap_, local_costmap_->getRobotFootprint(), escaped);
  new_cost = initial_cost;

  std::random_device rd;
  std::mt19937 g(rd());
  std::vector<int> lin_v(3), ang_v(11);
  std::iota(lin_v.begin(), lin_v.end(), -1);  // -1 .. +1 range; will translate into -0.1 .. +0.1 m/s
  std::iota(ang_v.begin(), ang_v.end(), -5);  // -5 .. +5 range; will translate into -0.5 .. +0.5 rad/s

  while (new_cost && !open_nodes.empty() && !cancel_requested_)
  {
    current_node = open_nodes.top();
    open_nodes.pop();

    if (current_node->cost_ == 0.0)
    {
      // Zero cost; we cannot get better, so do no search more. Anyway, as we are escaping from a difficult
      // situation, this is very suspicious... but could be that the obstacle blocking us vanished
      ROS_WARN_THROTTLE(1.0, "ZERO COST! (ignore if we are actually in an empty area)");
      best_node = current_node;
      new_cost = 0.0;
      break;
    }

    if (current_node->depth_ == max_search_depth_)
    {
      ROS_DEBUG("[%s] Reached maximum search depth (%d), search is finished!", name_.c_str(), max_search_depth_);
      break;
    }

    // Randomize commands order to avoid preferring any particular direction
    std::shuffle(lin_v.begin(), lin_v.end(), g);
    std::shuffle(ang_v.begin(), ang_v.end(), g);

    for (int lin : lin_v)
    {
      // Respect safety-restricted driving direction up to release_bumper_depth_ levels; after that many commands,
      // we expect that the bumper will be released, and so we are free to move in both directions (note that we
      // are planning ahead, and so drive_for/backward_allowed_ will remain false for the direction for which bumper
      if (best_node->depth_ <= release_bumper_depth_ &&
          ((!drive_forward_allowed_ && lin >= 0) || (!drive_backward_allowed_ && lin <= 0)))
        continue;

      for (int ang : ang_v)
      {
        double w = ang * w_max_ * 0.2;                    // scale down to -w_max_/+w_max_
        double v = lin * v_max_ * std::cos(ang * 0.2);    // scale down to -v_max_/+v_max_
                                                          // the cos restricts fastest wheel rotation to +v/-v max

        // Whenever the safety system is constraining the moving direction of the robot, this also discards angular
        // velocity commands that would result in a wheel turning in opposite direction (because rotation is too fast
        // for the linear speed; note on lin loop that rotations in place are prevented by discarding lin = 0)
        double max_w = (2 * std::abs(v)) / wheel_separation_;
        if (best_node->depth_ <= release_bumper_depth_ &&
            (!drive_forward_allowed_ || !drive_backward_allowed_) && std::abs(w) > max_w)
          continue;

        if (v == -current_node->cmd_vel_.linear.x && w == -current_node->cmd_vel_.angular.z)
        {
          // Prevent reverting previous command; not the best anti-oscillation method but it works
          // TODO: should instead check that we don't revisit a pose with the current commands chain
          ROS_DEBUG("[%s] REVERTING OSCILLATION %f == %f  &&  %f == %f", name_.c_str(),
                    v, - current_node->cmd_vel_.linear.x, w, - current_node->cmd_vel_.angular.z);
          continue;
        }

        double th = current_node->heading_ + w * time_step;
        double x =  current_node->pose_x_ + v * std::cos(th) * time_step;
        double y =  current_node->pose_y_ + v * std::sin(th) * time_step;
        double cost = getPoseCost(x, y, th, local_costmap_, local_costmap_->getRobotFootprint(), escaped);

        cmd_vel.linear.x = v;
        cmd_vel.angular.z = w;

        CmdVelNode* next_node = new CmdVelNode(current_node->depth_ + 1, x, y, th, cmd_vel, current_node, cost);
        if (cost < new_cost)
        {
          ROS_DEBUG("[%s] New best node found with cost: %f", name_.c_str(), new_cost);
          best_node = next_node;
          new_cost = cost;
        }

        current_node->following_.push_back(next_node);
        open_nodes.push(next_node);
        ROS_DEBUG("[%s] Open states: %lu; adding node at: %.2f, %.2f with cmd vel: %.2f, %.2f and depth: %d",
                  name_.c_str(), open_nodes.size(), next_node->pose_x_, next_node->pose_y_,
                  next_node->cmd_vel_.linear.x, next_node->cmd_vel_.angular.z, next_node->depth_);
      }
    }
  }

  nav_msgs::Path path;
  path.header.frame_id = robot_pose.header.frame_id;

  // don't bother to create a commands chain if we are canceling the behavior
  if (cancel_requested_)
  {
    // clear any previous path
    path_pub_.publish(path);
    return PlanResult::CANCELED;
  }

  ROS_DEBUG("[%s] SEARCH FINISHED! Best node reaches: %.2f, %.2f with cmd vel: %.2f, %.2f; depth: %d; " \
            "cost: %f (initial: %f); search time: %f", name_.c_str(), best_node->pose_x_, best_node->pose_y_,
            best_node->cmd_vel_.linear.x, best_node->cmd_vel_.angular.z, best_node->depth_, new_cost, initial_cost,
            (ros::Time::now() - t0).toSec());

  current_node = best_node;

  geometry_msgs::PoseStamped traj_pose(robot_pose);

  // build commands chain
  cmd_vels.clear();
  // every node translate into several velocity commands, as time_step is normally bigger than controller period
  int cmds_per_node = std::round(time_step * controller_frequency_);
  while (current_node->prev_)
  {
    for (int i = 0; i < cmds_per_node; ++i)
      cmd_vels.push_back(current_node->cmd_vel_);
    traj_pose.pose.position.x = current_node->pose_x_;
    traj_pose.pose.position.y = current_node->pose_y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, current_node->heading_);
    tf2::convert(q, traj_pose.pose.orientation);
    path.poses.push_back(traj_pose);
    current_node = current_node->prev_;
  }

  // clean up expanded states
  root_node->cleanup();

  std::reverse(cmd_vels.begin(), cmd_vels.end());
  std::reverse(path.poses.begin(), path.poses.end());

  if (cmd_vels.empty() || (cmd_vels.front().linear.x == 0.0 && cmd_vels.front().angular.z == 0.0))
  {
    double v = cmd_vels.empty() ? 0.0 : cmd_vels.front().linear.x;
    double w = cmd_vels.empty() ? 0.0 : cmd_vels.front().angular.z;
    // Best option is a zero velocity command; if happens for a while, we assume we reached a cost local minimum
    ROS_DEBUG("[%s] Zero command:  %f, %f   cost: %f <= %f  dt: %f     ALL?  %d     time %f", name_.c_str(), v, w,
              new_cost, prev_cost, time_step,
              std::all_of(cmd_vels.begin(), cmd_vels.end(),
                          [&](const geometry_msgs::Twist& cv) {return cv.linear.x == 0.0 && cv.angular.z == 0.0;}),
              (ros::Time::now() - t0).toSec());
    return new_cost != 0.0 ? PlanResult::ZERO_VEL : PlanResult::ZERO_COST;
  }

  if (new_cost == 0.0 || new_cost < prev_cost)
  {
    ROS_DEBUG("[%s] Valid plan with %lu commands;  first: %f, %f   cost: %f <= %f   time: %f", name_.c_str(),
              cmd_vels.size(), cmd_vels.front().linear.x, cmd_vels.front().angular.z, new_cost, prev_cost,
              (ros::Time::now() - t0).toSec());
    path_pub_.publish(path);
    return new_cost > 0.0 ? PlanResult::DEC_COST : PlanResult::ZERO_COST;
  }

  ROS_DEBUG("[%s] Cost inc --> bad plan with %lu commands;  first: %f, %f   cost: %f <= %f   time: %f", name_.c_str(),
            cmd_vels.size(), cmd_vels.front().linear.x, cmd_vels.front().angular.z, new_cost, prev_cost,
            (ros::Time::now() - t0).toSec());

  return PlanResult::INC_COST;
}

double SlowEscapeRecovery::getPoseCost(double x, double y, double th, costmap_2d::Costmap2DROS* costmap,
                                       const std::vector<geometry_msgs::Point>& footprint, bool& escaped)
{
  // Lock costmap mutex to ensure we don't read it while it gets updated
  boost::lock_guard<costmap_2d::Costmap2D::mutex_t> lg(*costmap->getCostmap()->getMutex());

  std::vector<base_local_planner::Position2DInt> cells =
      footprint_helper_.getFootprintCells(Eigen::Vector3f(x, y, th), footprint, *costmap->getCostmap(), true);

  double total_cost = 0.0;
  for (const base_local_planner::Position2DInt& cell : cells)
  {
    uint32_t cost = costmap->getCostmap()->getCost(cell.x, cell.y);
    switch (cost)
    {
      case costmap_2d::NO_INFORMATION:
        cost *= 100;
        break;
      case costmap_2d::LETHAL_OBSTACLE:
        if (escape_from_collision_ >= 0.0)
          escaped = false;
        cost *= 1000;
        break;
      case costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
        if (escape_from_inscribed_ >= 0.0)
          escaped = false;
        cost *= 10;
        break;
    }
    total_cost += cost;
  }
  return total_cost;
}

}; // namespace thorp_mbf_plugins
