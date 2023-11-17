#include "thorp_toolkit/progress_tracker.hpp"

#include <thorp_toolkit/geometry.hpp>

namespace thorp_toolkit
{

// TODO implement
class Visualization {
  // Placeholder for Visualization class in C++
public:
  void addDiscMarker(const geometry_msgs::PoseStamped& pose, const std::tuple<double, double, double>& color, const std::tuple<double, double, double, double>& dimensions) {
    // Implementation for adding a disc marker
  }

  void publishMarkers() {
    // Implementation for publishing markers
  }

  void deleteMarkers() {
    // Implementation for deleting markers
  }
};

ProgressTracker::ProgressTracker(const std::vector<geometry_msgs::PoseStamped>& waypoints, double reached_threshold)
  : _waypoints(waypoints)
  , _reached_threshold(reached_threshold)
  , _next_wp(0)
  , _reached(false)
  , _min_dist(std::numeric_limits<double>::infinity())
{
  if (_waypoints.empty())
  {
    throw std::invalid_argument("Waypoints must be a non-empty list of poses");
  }

  // Visualize semi-transparent waypoints; will become solid once reached
  for (const auto& wp : _waypoints)
  {
    Visualization().addDiscMarker(wp, std::make_tuple(0.2, 0.2, 0.000001), std::make_tuple(0, 0, 1.0, 0.25));
  }
  Visualization().publishMarkers();
}

void ProgressTracker::reset()
{
  _next_wp = 0;
  _reached = false;
  _min_dist = std::numeric_limits<double>::infinity();
  Visualization().deleteMarkers();
}

void ProgressTracker::updatePose(const geometry_msgs::PoseStamped& robot_pose)
{
  if (_next_wp == std::numeric_limits<size_t>::max())
  {
    return;  // already arrived
  }

  double dist = distance2D(robot_pose, _waypoints[_next_wp]);
  if (_reached && dist > _min_dist)
  {
    Visualization().addDiscMarker(_waypoints[_next_wp], std::make_tuple(0.2, 0.2, 0.000001), std::make_tuple(0, 0, 1.0, 1.0));
    Visualization().publishMarkers();
    _next_wp += 1;

    if (_next_wp < _waypoints.size())
    {
      // go for the next waypoint
      _reached = false;
      _min_dist = std::numeric_limits<double>::infinity();
    }
    else
    {
      // arrived; clear reached waypoints markers
      Visualization().deleteMarkers();
      _next_wp = std::numeric_limits<size_t>::max();
    }
    return;
  }

  _min_dist = std::min(_min_dist, dist);
  if (dist <= _reached_threshold)
  {
    _reached = true;
  }
}

size_t ProgressTracker::nextWaypoint() const
{
  return _next_wp;
}

size_t ProgressTracker::reachedWaypoint() const
{
  if (_next_wp == std::numeric_limits<size_t>::max())
  {
    return _waypoints.size() - 1;
  }
  else if (_next_wp == 0)
  {
    return std::numeric_limits<size_t>::max();
  }
  return _next_wp - 1;
}

}  // namespace thorp_toolkit
