#include "thorp_toolkit/progress_tracker.hpp"

#include <limits>
#include <stdexcept>

#include <thorp_toolkit/geometry.hpp>

namespace thorp::toolkit
{

ProgressTracker::ProgressTracker() : viz_("waypoints")
{
}

void ProgressTracker::init(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints, double reached_threshold)
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
  showWaypoints();
}

void ProgressTracker::reset()
{
  waypoints_.clear();
  next_wp_ = 0;
  reached_ = false;
  min_dist_ = std::numeric_limits<double>::infinity();
  viz_.reset();
}

void ProgressTracker::updatePose(const geometry_msgs::msg::PoseStamped& robot_pose)
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
    next_wp_ += 1;

    if (next_wp_ < waypoints_.size())
    {
      // go for the next waypoint
      reached_ = false;
      min_dist_ = std::numeric_limits<double>::infinity();
      showWaypoints();
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

void ProgressTracker::showWaypoints()
{
  // one marker per waypoint, so they keep their ids and RViz updates them in place
  viz_.clearMarkers();
  for (size_t i = 0; i < waypoints_.size(); ++i)
  {
    const double diameter = i == next_wp_ ? 2.0 * reached_threshold_ : 0.2;
    viz_.addDiscMarker(waypoints_[i], diameter, makeColor(0, 0, 1.0, i < next_wp_ ? 1.0 : 0.25));
  }
  viz_.publishMarkers();
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
