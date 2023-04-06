#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "thorp_costmap_layers/spatial_hash.hpp"

namespace thorp_costmap_layers
{
class Visualization
{
public:
  explicit Visualization(ros::NodeHandle& pnh);

  ~Visualization();

  void showUpdatedBounds(double old_min_x, double old_min_y, double old_max_x, double old_max_y,
                         double new_min_x, double new_min_y, double new_max_x, double new_max_y,
                         const std::string& frame_id);

  void showObjectContour(const Object& obj, const std::string& frame_id);

  void hideObjectContour(const Object& obj);

private:
  ros::Publisher debug_markers_pub_;  ///< publish updated bounds and other line strips for debugging

  void showLineStrip(const std::list<Point2d>& points, const std::string& frame_id, int id, const std::string& ns,
                     const std::string& color, float alpha = 1.0f);
};

}  // namespace thorp_costmap_layers
