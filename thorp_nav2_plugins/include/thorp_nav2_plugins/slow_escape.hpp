/*
 * Author: Jorge Santos
 */

#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_behaviors/timed_behavior.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_subscriber.hpp>
#include <nav2_costmap_2d/footprint_subscriber.hpp>
#include <nav_msgs/msg/path.hpp>

#include "thorp_nav2_plugins/action/slow_escape.hpp"

namespace thorp::nav2_plugins
{
/**
 * Escape slowly from where the planner or the controller can't move the robot, as inside an obstacle or its
 * inflation, following the costmaps' cost gradient down. Every cycle, a best-first search over slow velocity commands
 * scores the poses they lead to by the summed cost of the local costmap cells under the footprint, with unknown,
 * inscribed and lethal cells weighing much more, and the robot follows the cheapest sequence found.
 */
class SlowEscape : public nav2_behaviors::TimedBehavior<thorp_nav2_plugins::action::SlowEscape>
{
public:
  using Action = thorp_nav2_plugins::action::SlowEscape;

  nav2_behaviors::ResultStatus onRun(const std::shared_ptr<const Action::Goal> command) override;
  nav2_behaviors::ResultStatus onCycleUpdate() override;

  nav2_core::CostmapInfoType getResourceInfo() override
  {
    return nav2_core::CostmapInfoType::BOTH;
  }

protected:
  void onConfigure() override;
  void onCleanup() override;
  void onActionCompletion(std::shared_ptr<Action::Result> result) override;

private:
  struct Pose2D
  {
    double x;
    double y;
    double yaw;
  };

  struct Command
  {
    double v;
    double w;
  };

  /// One of the costmaps the behavior server receives, with the footprint it publishes
  struct Costmap
  {
    std::string frame;
    std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap;
    std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint;
    std::vector<geometry_msgs::msg::Point> robot_footprint;  // on the robot frame, padded with the clearance
  };

  enum class PlanResult
  {
    ZERO_COST,
    ZERO_VEL,
    INC_COST,
    DEC_COST
  };

  /**
   * Search the cheapest sequence of commands from a pose, projecting each command for time_step.
   * @param new_cost Cost at the end of the sequence found
   * @return How the sequence compares with prev_cost
   */
  PlanResult makePlan(nav2_costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::msg::Point>& footprint,
                      const Pose2D& start, double time_step, double prev_cost,
                      std::chrono::steady_clock::time_point deadline, double& new_cost,
                      std::vector<Command>& commands, nav_msgs::msg::Path& path);

  /// Whether no cell under the footprint is lethal, nor inscribed if escaping to free space
  bool escaped(nav2_costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::msg::Point>& footprint,
               const Pose2D& pose) const;

  /// Summed cost of the cells under the footprint, with unknown, inscribed and lethal cells weighing much more
  double poseCost(nav2_costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::msg::Point>& footprint,
                  const Pose2D& pose) const;

  std::vector<nav2_costmap_2d::MapLocation> cellsUnder(nav2_costmap_2d::Costmap2D& costmap,
                                                       const std::vector<geometry_msgs::msg::Point>& footprint,
                                                       const Pose2D& pose) const;

  bool getPose(const std::string& frame, Pose2D& pose) const;

  nav2_behaviors::ResultStatus fail(uint16_t error_code, const std::string& message);

  // Parameters
  double v_max_;
  double w_max_;
  double vel_projection_time_;
  int max_search_depth_;

  // Goal
  uint8_t mode_;
  double max_distance_;
  rclcpp::Time end_time_;

  // Execution
  Costmap local_;
  Costmap global_;
  Pose2D start_;
  std::deque<Command> commands_;
  double cost_;
  int stagnated_;
  double distance_traveled_;
  std::string error_msg_;
  std::mt19937 random_engine_{ std::random_device{}() };
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
};

}  // namespace thorp::nav2_plugins
