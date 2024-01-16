#include "thorp_toolkit/visualization.hpp"

namespace thorp::toolkit
{
// Initialization of the static member variable
// template <>
// Visualization* Singleton<Visualization>::singletonInstance = nullptr;

Visualization::Visualization(const std::string& topic, double lifetime) : lifetime_(lifetime)
{
  markers_pub_ = ros::NodeHandle("~").advertise<visualization_msgs::MarkerArray>(topic, 1);
  //  ros::Duration(0.25).sleep();  // wait a moment until the publisher is ready
}

void Visualization::publishMarkers(int start_id)
{
  if (ma_.markers.empty())
  {
    return;
  }
  ros::Duration duration(lifetime_);
  for (size_t i = 0; i < ma_.markers.size(); ++i)
  {
    ma_.markers[i].lifetime = duration;
    ma_.markers[i].id = start_id + i;
  }
  /*  if (active_markers_.size() > markers_.markers.size())
    {
      size_t offset = active_markers_.size() - markers_.markers.size();
      for (size_t i = offset; i < active_markers_.size(); ++i)
      {
        active_markers_[i].action = visualization_msgs::Marker::DELETE;
      }
      markers_pub_.publish(std::vector<visualization_msgs::Marker>(markers_.markers.begin(), markers_.markers.end())
                             .insert(active_markers_.begin(), active_markers_.begin() + offset, active_markers_.end()));
    }
    else  TODO makes any sense???  */
  {
    markers_pub_.publish(ma_);
  }
  //  active_markers_ = markers_.markers;
}

void Visualization::reset()
{
  deleteMarkers();
  ma_.markers.clear();
}

void Visualization::deleteMarkers()
{
  visualization_msgs::MarkerArray ma;
  ma.markers.resize(1);
  ma.markers.front().action = visualization_msgs::Marker::DELETEALL;
  markers_pub_.publish(ma);
  //  active_markers_.clear();
}

void Visualization::clearMarkers()
{
  ma_.markers.clear();
}

void Visualization::addMarkers(const std::vector<visualization_msgs::Marker>& markers)
{
  ma_.markers.insert(ma_.markers.end(), markers.begin(), markers.end());
}

void Visualization::addTextMarker(const geometry_msgs::PoseStamped& pose, const std::string& text, double size,
                                  const std_msgs::ColorRGBA& color)
{
  ma_.markers.push_back(createTextMarker(pose, text, size, color));
}

void Visualization::addPointMarkers(const std::vector<geometry_msgs::PoseStamped>& poses, double size,
                                    const std_msgs::ColorRGBA& color)
{
  std::vector<visualization_msgs::Marker> point_markers = createPointMarkers(poses, size, color);
  ma_.markers.insert(ma_.markers.end(), point_markers.begin(), point_markers.end());
}

void Visualization::addLineMarker(const std::vector<geometry_msgs::PoseStamped>& poses, double size,
                                  const std_msgs::ColorRGBA& color)
{
  std::vector<geometry_msgs::Point> points;
  for (const auto& pose : poses)
  {
    points.push_back(pose.pose.position);
  }
  ma_.markers.push_back(createLineMarker(points, poses[0].header, size, color));
}

void Visualization::addBoxMarker(const geometry_msgs::PoseStamped& pose, const std::array<double, 3>& dimensions,
                                 const std_msgs::ColorRGBA& color)
{
  ma_.markers.push_back(createGeometryPrimitiveMarker(pose, dimensions, color, visualization_msgs::Marker::CUBE));
}

void Visualization::addDiscMarker(const geometry_msgs::PoseStamped& pose, double diameter,
                                  const std_msgs::ColorRGBA& color)
{
  std::array<double, 3> dimensions{ diameter, diameter, 1e-6 };
  ma_.markers.push_back(createGeometryPrimitiveMarker(pose, dimensions, color, visualization_msgs::Marker::CYLINDER));
}

visualization_msgs::Marker Visualization::createMeshMarker(const geometry_msgs::PoseStamped& pose,
                                                           const std::string& mesh_file,
                                                           const std::vector<double>& scale,
                                                           const std_msgs::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_file;
  marker.ns = ns;
  marker.scale.x = (scale.size() > 0) ? scale[0] : 1.0;
  marker.scale.y = (scale.size() > 1) ? scale[1] : 1.0;
  marker.scale.z = (scale.size() > 2) ? scale[2] : 1.0;
  marker.color = color;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header = pose.header;
  marker.pose = pose.pose;
  return marker;
}

visualization_msgs::Marker Visualization::createGeometryPrimitiveMarker(const geometry_msgs::PoseStamped& pose,
                                                                        const std::array<double, 3>& dimensions,
                                                                        const std_msgs::ColorRGBA& color, int shape,
                                                                        const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.type = shape;
  marker.ns = ns;
  marker.scale.x = std::max(dimensions[0], MIN_SCALE);
  marker.scale.y = std::max(dimensions[1], MIN_SCALE);
  marker.scale.z = std::max(dimensions[2], MIN_SCALE);
  marker.color = color;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header = pose.header;
  marker.pose = pose.pose;
  return marker;
}

visualization_msgs::Marker Visualization::createLineMarker(const std::vector<geometry_msgs::Point>& points,
                                                           const std_msgs::Header& header, double length,
                                                           const std_msgs::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.ns = ns;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = length;
  marker.color = color;
  marker.points = points;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header = header;
  return marker;
}

visualization_msgs::Marker Visualization::createPointMarker(const geometry_msgs::PoseStamped& pose, double size,
                                                            const std_msgs::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.header = pose.header;
  marker.pose = pose.pose;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color = color;
  marker.action = visualization_msgs::Marker::ADD;
  return marker;
}

visualization_msgs::Marker Visualization::createTextMarker(const geometry_msgs::PoseStamped& poseSt,
                                                           const std::string& text, double size,
                                                           const std_msgs::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.header = poseSt.header;
  marker.pose = poseSt.pose;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = ns;
  marker.text = text;
  marker.color = color;
  marker.scale.z = size;
  return marker;
}

std::vector<visualization_msgs::Marker>
Visualization::createPointMarkers(const std::vector<geometry_msgs::PoseStamped>& poses, double size,
                                  const std_msgs::ColorRGBA& color, const std::string& ns)
{
  std::vector<visualization_msgs::Marker> markers;

  for (const auto& pose : poses)
  {
    visualization_msgs::Marker marker;
    marker.ns = ns;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.header = pose.header;
    marker.pose = pose.pose;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color = color;
    marker.action = visualization_msgs::Marker::ADD;
    markers.push_back(marker);
  }

  return markers;
}

visualization_msgs::Marker Visualization::createCubeList(const geometry_msgs::PoseStamped& ref_pose,
                                                         const std::vector<geometry_msgs::Point>& poses,
                                                         const std::vector<std_msgs::ColorRGBA>& colors, double size,
                                                         const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = ns;
  marker.header = ref_pose.header;
  marker.pose = ref_pose.pose;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.points = poses;
  marker.colors = colors;
  return marker;
}

visualization_msgs::Marker Visualization::createPointList(const geometry_msgs::PoseStamped& ref_pose,
                                                          const std::vector<geometry_msgs::Point>& points,
                                                          const std::vector<std_msgs::ColorRGBA>& colors, double size,
                                                          const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.header = ref_pose.header;
  marker.pose = ref_pose.pose;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.points = points;
  marker.colors = colors;
  return marker;
}

jsk_rviz_plugins::OverlayText Visualization::createOverlayText(int offsetFromTop, const std_msgs::ColorRGBA& color,
                                                               const std::string& text, double text_size)
{
  jsk_rviz_plugins::OverlayText msg;
  msg.action = jsk_rviz_plugins::OverlayText::ADD;
  msg.width = 1000;
  msg.height = 25;
  msg.left = 5;
  msg.top = offsetFromTop;
  msg.fg_color = color;
  msg.text = text;
  msg.text_size = text_size;
  return msg;
}

} /* namespace thorp::toolkit */
