#pragma once

#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <thorp_toolkit/visualization.hpp>

namespace thorp::toolkit
{
/**
 * Track progress along a list of waypoints, so we can report the last crossed waypoint.
 * Clients must continuously update the robot pose without significant jumps.
 */

class ProgressTracker
{
public:
  ProgressTracker(const std::vector<geometry_msgs::PoseStamped>& waypoints, double reached_threshold);

  void reset();

  void updatePose(const geometry_msgs::PoseStamped& robot_pose);

  size_t nextWaypoint() const;

  size_t reachedWaypoint() const;

private:
  std::vector<geometry_msgs::PoseStamped> _waypoints;
  double reached_threshold;
  size_t next_wp;
  bool reached;
  double min_dist;

  Visualization viz;
};

}  // namespace thorp::toolkit
