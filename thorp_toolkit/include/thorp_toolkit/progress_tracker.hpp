#pragma once

#include <vector>

#include <geometry_msgs/PoseStamped.h>

namespace thorp_toolkit
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
  double _reached_threshold;
  size_t _next_wp;
  bool _reached;
  double _min_dist;
};

}  // namespace thorp_toolkit
