#include "thorp_toolkit/progress_tracker.hpp"

#include <thorp_toolkit/geometry.hpp>

namespace thorp::toolkit
{

ProgressTracker::ProgressTracker(const std::vector<geometry_msgs::PoseStamped>& waypoints, double reached_threshold)
  : _waypoints(waypoints)
  , reached_threshold(reached_threshold)
  , next_wp(0)
  , reached(false)
  , min_dist(std::numeric_limits<double>::infinity())
  , viz("visual_markers")
{
  if (_waypoints.empty())
  {
    throw std::invalid_argument("Waypoints must be a non-empty list of poses");
  }

  // Visualize semi-transparent waypoints; will become solid once reached
  for (const auto& wp : _waypoints)
  {
    viz.addDiscMarker(wp, 0.2, makeColor(0, 0, 1.0, 0.25));
  }
  viz.publishMarkers();
}

void ProgressTracker::reset()
{
  next_wp = 0;
  reached = false;
  min_dist = std::numeric_limits<double>::infinity();
  viz.deleteMarkers();
}

void ProgressTracker::updatePose(const geometry_msgs::PoseStamped& robot_pose)
{
  if (next_wp == std::numeric_limits<size_t>::max())
  {
    return;  // already arrived
  }

  double dist = distance2D(robot_pose, _waypoints[next_wp]);
  if (reached && dist > min_dist)
  {
    viz.addDiscMarker(_waypoints[next_wp], 0.2, makeColor(0, 0, 1.0, 1.0));
    viz.publishMarkers();
    next_wp += 1;

    if (next_wp < _waypoints.size())
    {
      // go for the next waypoint
      reached = false;
      min_dist = std::numeric_limits<double>::infinity();
    }
    else
    {
      // arrived; clear reached waypoints markers
      viz.deleteMarkers();
      next_wp = std::numeric_limits<size_t>::max();
    }
    return;
  }

  min_dist = std::min(min_dist, dist);
  if (dist <= reached_threshold)
  {
    reached = true;
  }
}

size_t ProgressTracker::nextWaypoint() const
{
  return next_wp;
}

size_t ProgressTracker::reachedWaypoint() const
{
  if (next_wp == std::numeric_limits<size_t>::max())
  {
    return _waypoints.size() - 1;
  }
  else if (next_wp == 0)
  {
    return std::numeric_limits<size_t>::max();
  }
  return next_wp - 1;
}

}  // namespace thorp::toolkit
