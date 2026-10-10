/*
 * Author: Jorge Santos
 */

#include "thorp_nav2_plugins/slow_escape.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_util/robot_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.hpp>

namespace thorp::nav2_plugins
{
namespace
{
// Cycles in a row without a command that lowers the cost, before taking it as a local minimum
constexpr int STAGNATION_LIMIT = 5;
// Share of the cycle period the search can take
constexpr double PLANNING_TIME_SHARE = 0.5;
// Bound to the search's memory, besides its depth and time
constexpr size_t MAX_SEARCH_NODES = 100000;
// Distance a robot that ends up stuck must have moved for a DISTANCE escape to count as done
constexpr double MIN_MOVEMENT = 0.01;

constexpr double UNKNOWN_WEIGHT = 100.0;
constexpr double INSCRIBED_WEIGHT = 10.0;
constexpr double LETHAL_WEIGHT = 1000.0;
}  // namespace

void SlowEscape::onConfigure()
{
  auto node = std::dynamic_pointer_cast<nav2_util::LifecycleNode>(node_.lock());
  if (!node)
    throw std::runtime_error("Failed to lock node");

  nav2_util::declare_parameter_if_not_declared(node, behavior_name_ + ".v_max", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, behavior_name_ + ".w_max", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, behavior_name_ + ".vel_projection_time",
                                               rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, behavior_name_ + ".max_search_depth", rclcpp::ParameterValue(30));
  node->get_parameter(behavior_name_ + ".v_max", v_max_);
  node->get_parameter(behavior_name_ + ".w_max", w_max_);
  node->get_parameter(behavior_name_ + ".vel_projection_time", vel_projection_time_);
  node->get_parameter(behavior_name_ + ".max_search_depth", max_search_depth_);

  // The behavior server's collision checkers only check footprints for collision, so we subscribe again to get costs
  std::string local_costmap_topic, global_costmap_topic, local_footprint_topic, global_footprint_topic;
  node->get_parameter("local_costmap_topic", local_costmap_topic);
  node->get_parameter("global_costmap_topic", global_costmap_topic);
  node->get_parameter("local_footprint_topic", local_footprint_topic);
  node->get_parameter("global_footprint_topic", global_footprint_topic);
  local_.frame = local_frame_;
  local_.costmap = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, local_costmap_topic);
  local_.footprint = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(node, local_footprint_topic, *tf_,
                                                                             robot_base_frame_, transform_tolerance_);
  global_.frame = global_frame_;
  global_.costmap = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, global_costmap_topic);
  global_.footprint = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(node, global_footprint_topic, *tf_,
                                                                              robot_base_frame_, transform_tolerance_);

  // Activated right away, as the behavior only runs while the server is active
  plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(behavior_name_ + "/plan", 1);
  plan_pub_->on_activate();
}

void SlowEscape::onCleanup()
{
  plan_pub_.reset();
  local_ = Costmap();
  global_ = Costmap();
}

nav2_behaviors::ResultStatus SlowEscape::onRun(const std::shared_ptr<const Action::Goal> command)
{
  error_msg_.clear();
  if (command->mode > Action::Goal::OUT_TO_FREE_SPACE || command->clearance < 0.0 || command->max_distance <= 0.0)
    return fail(Action::Result::INVALID_INPUT,
                "Invalid goal: mode " + std::to_string(command->mode) + ", clearance " +
                    std::to_string(command->clearance) + ", max distance " + std::to_string(command->max_distance));
  mode_ = command->mode;
  max_distance_ = command->max_distance;
  const rclcpp::Duration time_allowance(command->time_allowance);
  end_time_ = time_allowance.nanoseconds() > 0 ? clock_->now() + time_allowance : rclcpp::Time(0, 0, RCL_ROS_TIME);

  for (Costmap* costmap : { &local_, &global_ })
  {
    std_msgs::msg::Header header;
    try
    {
      costmap->costmap->getCostmap();
    }
    catch (const std::runtime_error&)
    {
      return fail(Action::Result::NO_COSTMAP, "No costmap received on the " + costmap->frame + " frame");
    }
    if (!costmap->footprint->getFootprintInRobotFrame(costmap->robot_footprint, header) ||
        costmap->robot_footprint.size() < 3)
      return fail(Action::Result::NO_COSTMAP, "No footprint received on the " + costmap->frame + " frame");
    nav2_costmap_2d::padFootprint(costmap->robot_footprint, command->clearance);
  }

  if (!getPose(global_frame_, start_))
    return fail(Action::Result::TF_ERROR, "Robot pose unavailable on " + global_frame_);

  commands_.clear();
  cost_ = std::numeric_limits<double>::infinity();
  stagnated_ = 0;
  distance_traveled_ = 0.0;
  RCLCPP_INFO(logger_, "%s: escaping %s, up to %.2f m", behavior_name_.c_str(),
              mode_ == Action::Goal::DISTANCE         ? "down the cost gradient" :
              mode_ == Action::Goal::OUT_OF_COLLISION ? "out of collision" :
                                                        "out to free space",
              max_distance_);
  return { nav2_behaviors::Status::SUCCEEDED, Action::Result::NONE };
}

nav2_behaviors::ResultStatus SlowEscape::onCycleUpdate()
{
  if (end_time_.nanoseconds() > 0 && clock_->now() > end_time_)
    return fail(Action::Result::TIMEOUT, "Time allowance exceeded");

  Pose2D local_pose, global_pose;
  if (!getPose(local_frame_, local_pose) || !getPose(global_frame_, global_pose))
    return fail(Action::Result::TF_ERROR, "Robot pose unavailable");

  distance_traveled_ = std::hypot(global_pose.x - start_.x, global_pose.y - start_.y);
  auto feedback = std::make_shared<Action::Feedback>();
  feedback->distance_traveled = static_cast<float>(distance_traveled_);
  action_server_->publish_feedback(feedback);

  auto local_costmap = local_.costmap->getCostmap();
  auto global_costmap = global_.costmap->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> local_lock(*local_costmap->getMutex());

  if (mode_ != Action::Goal::DISTANCE && escaped(*local_costmap, local_.robot_footprint, local_pose))
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> global_lock(*global_costmap->getMutex());
    if (escaped(*global_costmap, global_.robot_footprint, global_pose))
    {
      stopRobot();
      RCLCPP_INFO(logger_, "%s: escaped after %.2f m", behavior_name_.c_str(), distance_traveled_);
      return { nav2_behaviors::Status::SUCCEEDED, Action::Result::NONE };
    }
  }

  if (distance_traveled_ >= max_distance_)
  {
    if (mode_ != Action::Goal::DISTANCE)
      return fail(Action::Result::TOO_FAR, "Travelled " + std::to_string(max_distance_) + " m without escaping");
    stopRobot();
    return { nav2_behaviors::Status::SUCCEEDED, Action::Result::NONE };
  }

  // The footprint is padded only to check whether we escaped; the search uses the robot's own
  std::vector<geometry_msgs::msg::Point> footprint;
  std_msgs::msg::Header header;
  local_.footprint->getFootprintInRobotFrame(footprint, header);

  // Stagnating, we project commands for longer, so they reach poses whose cost differs
  const double time_step = vel_projection_time_ * (1.0 + stagnated_ * 0.2);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(PLANNING_TIME_SHARE / cycle_frequency_));
  double new_cost;
  std::vector<Command> new_commands;
  nav_msgs::msg::Path path;
  const PlanResult result =
      makePlan(*local_costmap, footprint, local_pose, time_step, cost_, deadline, new_cost, new_commands, path);
  local_lock.unlock();

  if ((result == PlanResult::INC_COST || result == PlanResult::ZERO_VEL) && commands_.empty())
    ++stagnated_;
  else
    stagnated_ = 0;
  if (stagnated_ > STAGNATION_LIMIT)
  {
    if (mode_ != Action::Goal::DISTANCE || distance_traveled_ < MIN_MOVEMENT)
      return fail(Action::Result::STUCK, "No command lowers the cost");
    stopRobot();
    RCLCPP_INFO(logger_, "%s: reached a cost minimum after %.2f m", behavior_name_.c_str(), distance_traveled_);
    return { nav2_behaviors::Status::SUCCEEDED, Action::Result::NONE };
  }

  if (commands_.empty() || new_cost < cost_)
  {
    commands_.assign(new_commands.begin(), new_commands.end());
    cost_ = new_cost;
    path.header.frame_id = local_frame_;
    path.header.stamp = clock_->now();
    plan_pub_->publish(path);
  }

  if (commands_.empty())
  {
    if (result == PlanResult::ZERO_COST)
    {
      stopRobot();
      RCLCPP_INFO(logger_, "%s: no cost left under the footprint", behavior_name_.c_str());
      return { nav2_behaviors::Status::SUCCEEDED, Action::Result::NONE };
    }
    stopRobot();
    return { nav2_behaviors::Status::RUNNING, Action::Result::NONE };
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.frame_id = robot_base_frame_;
  cmd_vel->header.stamp = clock_->now();
  cmd_vel->twist.linear.x = commands_.front().v;
  cmd_vel->twist.angular.z = commands_.front().w;
  vel_pub_->publish(std::move(cmd_vel));
  commands_.pop_front();
  return { nav2_behaviors::Status::RUNNING, Action::Result::NONE };
}

void SlowEscape::onActionCompletion(std::shared_ptr<Action::Result> result)
{
  result->distance_traveled = static_cast<float>(distance_traveled_);
  result->error_msg = error_msg_;
  commands_.clear();
}

SlowEscape::PlanResult SlowEscape::makePlan(nav2_costmap_2d::Costmap2D& costmap,
                                            const std::vector<geometry_msgs::msg::Point>& footprint,
                                            const Pose2D& start, double time_step, double prev_cost,
                                            std::chrono::steady_clock::time_point deadline, double& new_cost,
                                            std::vector<Command>& commands, nav_msgs::msg::Path& path)
{
  struct Node
  {
    Pose2D pose;
    Command command;
    int linear;   // command steps, to tell a command that reverts the previous one
    int angular;
    int depth;
    int previous;
    double cost;
  };
  std::vector<Node> nodes;
  auto costlier = [&nodes](int a, int b) { return nodes[a].cost > nodes[b].cost; };
  std::priority_queue<int, std::vector<int>, decltype(costlier)> open(costlier);

  nodes.push_back({ start, { 0.0, 0.0 }, 0, 0, 0, -1, poseCost(costmap, footprint, start) });
  open.push(0);
  int best = 0;
  new_cost = nodes[0].cost;

  std::array<int, 3> linear_steps;
  std::array<int, 11> angular_steps;
  std::iota(linear_steps.begin(), linear_steps.end(), -1);
  std::iota(angular_steps.begin(), angular_steps.end(), -5);

  while (new_cost > 0.0 && !open.empty() && nodes.size() < MAX_SEARCH_NODES &&
         std::chrono::steady_clock::now() < deadline)
  {
    const Node current = nodes[open.top()];
    const int current_index = open.top();
    open.pop();
    if (current.cost == 0.0)
    {
      best = current_index;
      new_cost = 0.0;
      break;
    }
    if (current.depth == max_search_depth_)
      break;

    // Shuffled, not to prefer any direction among equally good commands
    std::shuffle(linear_steps.begin(), linear_steps.end(), random_engine_);
    std::shuffle(angular_steps.begin(), angular_steps.end(), random_engine_);
    for (int linear : linear_steps)
    {
      for (int angular : angular_steps)
      {
        // Reverting the previous command makes the search oscillate
        if (linear == -current.linear && angular == -current.angular)
          continue;

        // The cosine slows down translation as rotation speeds up, so neither wheel exceeds v_max
        const double w = angular * w_max_ * 0.2;
        const double v = linear * v_max_ * std::cos(angular * 0.2);
        Pose2D pose;
        pose.yaw = current.pose.yaw + w * time_step;
        pose.x = current.pose.x + v * std::cos(pose.yaw) * time_step;
        pose.y = current.pose.y + v * std::sin(pose.yaw) * time_step;
        const double cost = poseCost(costmap, footprint, pose);
        nodes.push_back({ pose, { v, w }, linear, angular, current.depth + 1, current_index, cost });
        if (cost < new_cost)
        {
          best = static_cast<int>(nodes.size()) - 1;
          new_cost = cost;
        }
        open.push(static_cast<int>(nodes.size()) - 1);
      }
    }
  }

  // Each node stands for several commands, as time_step is longer than our cycle
  const int commands_per_node = std::max(1, static_cast<int>(std::round(time_step * cycle_frequency_)));
  commands.clear();
  path.poses.clear();
  for (int i = best; nodes[i].previous >= 0; i = nodes[i].previous)
  {
    commands.insert(commands.end(), commands_per_node, nodes[i].command);
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = nodes[i].pose.x;
    pose.pose.position.y = nodes[i].pose.y;
    pose.pose.orientation.z = std::sin(nodes[i].pose.yaw / 2.0);
    pose.pose.orientation.w = std::cos(nodes[i].pose.yaw / 2.0);
    path.poses.push_back(pose);
  }
  std::reverse(commands.begin(), commands.end());
  std::reverse(path.poses.begin(), path.poses.end());

  if (commands.empty())
    return new_cost == 0.0 ? PlanResult::ZERO_COST : PlanResult::ZERO_VEL;
  if (new_cost == 0.0)
    return PlanResult::ZERO_COST;
  return new_cost < prev_cost ? PlanResult::DEC_COST : PlanResult::INC_COST;
}

bool SlowEscape::escaped(nav2_costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::msg::Point>& footprint,
                         const Pose2D& pose) const
{
  for (const auto& cell : cellsUnder(costmap, footprint, pose))
  {
    const unsigned char cost = costmap.getCost(cell.x, cell.y);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        (mode_ == Action::Goal::OUT_TO_FREE_SPACE && cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
      return false;
  }
  return true;
}

double SlowEscape::poseCost(nav2_costmap_2d::Costmap2D& costmap,
                            const std::vector<geometry_msgs::msg::Point>& footprint, const Pose2D& pose) const
{
  double total = 0.0;
  for (const auto& cell : cellsUnder(costmap, footprint, pose))
  {
    const unsigned char cost = costmap.getCost(cell.x, cell.y);
    switch (cost)
    {
      case nav2_costmap_2d::NO_INFORMATION:
        total += cost * UNKNOWN_WEIGHT;
        break;
      case nav2_costmap_2d::LETHAL_OBSTACLE:
        total += cost * LETHAL_WEIGHT;
        break;
      case nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
        total += cost * INSCRIBED_WEIGHT;
        break;
      default:
        total += cost;
    }
  }
  return total;
}

std::vector<nav2_costmap_2d::MapLocation>
SlowEscape::cellsUnder(nav2_costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::msg::Point>& footprint,
                       const Pose2D& pose) const
{
  std::vector<nav2_costmap_2d::MapLocation> polygon, cells;
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  for (const auto& point : footprint)
  {
    int mx, my;
    costmap.worldToMapEnforceBounds(pose.x + point.x * cos_yaw - point.y * sin_yaw,
                                    pose.y + point.x * sin_yaw + point.y * cos_yaw, mx, my);
    polygon.push_back({ static_cast<unsigned int>(mx), static_cast<unsigned int>(my) });
  }
  costmap.convexFillCells(polygon, cells);
  return cells;
}

bool SlowEscape::getPose(const std::string& frame, Pose2D& pose) const
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  if (!nav2_util::getCurrentPose(pose_stamped, *tf_, frame, robot_base_frame_, transform_tolerance_))
    return false;
  pose = { pose_stamped.pose.position.x, pose_stamped.pose.position.y, tf2::getYaw(pose_stamped.pose.orientation) };
  return true;
}

nav2_behaviors::ResultStatus SlowEscape::fail(uint16_t error_code, const std::string& message)
{
  stopRobot();
  error_msg_ = message;
  RCLCPP_WARN(logger_, "%s: %s", behavior_name_.c_str(), message.c_str());
  return { nav2_behaviors::Status::FAILED, error_code };
}

}  // namespace thorp::nav2_plugins

PLUGINLIB_EXPORT_CLASS(thorp::nav2_plugins::SlowEscape, nav2_core::Behavior)
