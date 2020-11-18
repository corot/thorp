#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

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

  void showLineStrip(const std::vector<geometry_msgs::Point>& points, int id, const std::string& frame_id,
                     const std::string& color, float alpha = 1.0f);

private:
  ros::Publisher debug_markers_pub_;   ///< publish updated bounds and other line strips for debugging

  geometry_msgs::Point makePointMsg(double x, double y);
};

}
