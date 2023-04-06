#include <thorp_toolkit/common.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_costmap_layers/visualization.hpp"

namespace thorp_costmap_layers
{

Visualization::Visualization(ros::NodeHandle& pnh)
{
  // Optionally, publish updated bounds and other line strips for debugging
  if (pnh.param<bool>("show_debug_markers", false))
    debug_markers_pub_ = pnh.advertise<visualization_msgs::Marker>("visual_markers", 1000);
}

Visualization::~Visualization() = default;

void Visualization::showUpdatedBounds(double old_min_x, double old_min_y, double old_max_x, double old_max_y,
                                      double new_min_x, double new_min_y, double new_max_x, double new_max_y,
                                      const std::string& frame_id)
{
  if (!debug_markers_pub_.getNumSubscribers())
    return;

  showLineStrip({ Point2d(new_min_x, new_min_y), Point2d(new_min_x, new_max_y), Point2d(new_max_x, new_max_y),
                  Point2d(new_max_x, new_min_y), Point2d(new_min_x, new_min_y) },
                frame_id, 1, "updated bounds", "white");
  showLineStrip({ Point2d(old_min_x, old_min_y), Point2d(old_min_x, old_max_y), Point2d(old_max_x, old_max_y),
                  Point2d(old_max_x, old_min_y), Point2d(old_min_x, old_min_y) },
                frame_id, 1, "previous bounds", "beige", 0.5f);
}

void Visualization::showObjectContour(const Object& obj, const std::string& frame_id)
{
  if (!debug_markers_pub_.getNumSubscribers())
    return;

  std::list<Point2d> points{ obj.contour_points };
  points.push_back(points.front());  // append first point to close the contour
  showLineStrip(points, frame_id, obj.id, "object contours", "yellow");
}

void Visualization::hideObjectContour(const Object& obj)
{
  if (!debug_markers_pub_.getNumSubscribers())
    return;

  visualization_msgs::Marker marker;
  marker.ns = "object contours";
  marker.id = obj.id;
  marker.action = visualization_msgs::Marker::DELETE;

  debug_markers_pub_.publish(marker);
}

void Visualization::showLineStrip(const std::list<Point2d>& points, const std::string& frame_id, int id,
                                  const std::string& ns, const std::string& color, float alpha)
{
  if (!debug_markers_pub_.getNumSubscribers())
    return;

  // optionally, publish updated and previous bounds as a markers, for debugging purposes
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.pose.orientation.w = 1.0;
  marker.color = ttk::namedColor(color, alpha);
  marker.scale.x = 0.05;
  std::transform(points.begin(), points.end(), std::back_inserter(marker.points),
                 [](const Point2d& pt) -> geometry_msgs::Point { return pt.toPointMsg(); });

  debug_markers_pub_.publish(marker);
}

}  // namespace thorp_costmap_layers
