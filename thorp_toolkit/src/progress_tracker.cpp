#include "thorp_toolkit/progress_tracker.hpp"

#include <thorp_toolkit/geometry.hpp>

namespace thorp::toolkit
{

ProgressTracker::ProgressTracker() : viz_("waypoints")
{
}

void ProgressTracker::init(const std::vector<geometry_msgs::PoseStamped>& waypoints, double reached_threshold)
{
  if (waypoints.empty())
  {
    throw std::invalid_argument("Waypoints must be a non-empty list of poses");
  }

  next_wp_ = 0;
  reached_ = false;
  min_dist_ = std::numeric_limits<double>::infinity();
  waypoints_ = waypoints;
  reached_threshold_ = reached_threshold;

  // Visualize semi-transparent waypoints; will become solid once reached_
  for (const auto& wp : waypoints_)
  {
    viz_.addDiscMarker(wp, 0.2, makeColor(0, 0, 1.0, 0.25));
  }
  viz_.publishMarkers();
}

void ProgressTracker::reset()
{
  waypoints_.clear();
  next_wp_ = 0;
  reached_ = false;
  min_dist_ = std::numeric_limits<double>::infinity();
  viz_.reset();
}

void ProgressTracker::updatePose(const geometry_msgs::PoseStamped& robot_pose)
{
  if (waypoints_.empty())
  {
    throw std::invalid_argument("Not initialized");
  }

  if (next_wp_ == std::numeric_limits<size_t>::max())
  {
    return;  // already arrived
  }

  double dist = distance2D(robot_pose, waypoints_[next_wp_]);
  if (reached_ && dist > min_dist_)
  {
    viz_.addDiscMarker(waypoints_[next_wp_], 0.2, makeColor(0, 0, 1.0, 1.0));
    viz_.publishMarkers();
    next_wp_ += 1;

    if (next_wp_ < waypoints_.size())
    {
      // go for the next waypoint
      reached_ = false;
      min_dist_ = std::numeric_limits<double>::infinity();
    }
    else
    {
      // arrived; clear reached waypoints markers
      viz_.deleteMarkers();
      next_wp_ = std::numeric_limits<size_t>::max();
    }
    return;
  }

  min_dist_ = std::min(min_dist_, dist);
  if (dist <= reached_threshold_)
  {
    reached_ = true;
  }
}

size_t ProgressTracker::nextWaypoint() const
{
  if (waypoints_.empty())
  {
    throw std::invalid_argument("Not initialized");
  }

  return next_wp_;
}

size_t ProgressTracker::reachedWaypoint() const
{
  if (waypoints_.empty())
  {
    throw std::invalid_argument("Not initialized");
  }

  if (next_wp_ == std::numeric_limits<size_t>::max())
  {
    return waypoints_.size() - 1;
  }
  else if (next_wp_ == 0)
  {
    return std::numeric_limits<size_t>::max();
  }
  return next_wp_ - 1;
}

}  // namespace thorp::toolkit
