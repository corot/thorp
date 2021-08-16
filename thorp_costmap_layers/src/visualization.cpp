#include <thorp_toolkit/common.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_costmap_layers/visualization.h"

namespace thorp_costmap_layers
{

Visualization::Visualization(ros::NodeHandle& pnh)
{
  // Optionally, publish updated bounds and other line strips for debugging
  if (pnh.param<bool>("show_debug_markers", false))
    debug_markers_pub_ = pnh.advertise<visualization_msgs::Marker>("updated_bounds_marker", 1);
}

Visualization::~Visualization()
{
}

void Visualization::showUpdatedBounds(double old_min_x, double old_min_y, double old_max_x, double old_max_y,
                                      double new_min_x, double new_min_y, double new_max_x, double new_max_y,
                                      const std::string& frame_id)
{
  if (!debug_markers_pub_.getNumSubscribers())
    return;

  showLineStrip({makePointMsg(new_min_x, new_min_y),
                 makePointMsg(new_min_x, new_max_y),
                 makePointMsg(new_max_x, new_max_y),
                 makePointMsg(new_max_x, new_min_y),
                 makePointMsg(new_min_x, new_min_y)}, 1, frame_id, "white");
  showLineStrip({makePointMsg(old_min_x, old_min_y),
                 makePointMsg(old_min_x, old_max_y),
                 makePointMsg(old_max_x, old_max_y),
                 makePointMsg(old_max_x, old_min_y),
                 makePointMsg(old_min_x, old_min_y)}, 2, frame_id, "gray", 0.5f);
}

void Visualization::showLineStrip(const std::vector<geometry_msgs::Point>& points, int id, const std::string& frame_id,
                                  const std::string& color, float alpha)
{
  if (!debug_markers_pub_.getNumSubscribers())
    return;

  // optionally, publish updated and previous bounds as a markers, for debugging purposes
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "semantic layer " + std::to_string(id);
  marker.id = id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.pose.orientation.w = 1.0;
  marker.color = ttk::namedColor(color, alpha);
  marker.scale.x = 0.05;
  marker.points = points;

  debug_markers_pub_.publish(marker);
}

geometry_msgs::Point Visualization::makePointMsg(double x, double y)
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  return point;
}

} // end namespace
