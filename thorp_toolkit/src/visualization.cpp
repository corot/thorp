#include "thorp_toolkit/visualization.hpp"

namespace thorp::toolkit
{
// Initialization of the static member variable
// template <>
// Visualization* Singleton<Visualization>::singletonInstance = nullptr;

Visualization::Visualization(const std::string& topic, double lifetime) : lifetime_(lifetime)
{
  // published on the toolkit node private namespace
  markers_pub_ = node()->create_publisher<visualization_msgs::msg::MarkerArray>("~/" + topic, 1);
}

void Visualization::publishMarkers(int start_id)
{
  if (ma_.markers.empty())
  {
    return;
  }
  builtin_interfaces::msg::Duration duration = rclcpp::Duration::from_seconds(lifetime_);
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
        active_markers_[i].action = visualization_msgs::msg::Marker::DELETE;
      }
      markers_pub_->publish(std::vector<visualization_msgs::msg::Marker>(markers_.markers.begin(), markers_.markers.end())
                             .insert(active_markers_.begin(), active_markers_.begin() + offset, active_markers_.end()));
    }
    else  TODO makes any sense???  */
  {
    markers_pub_->publish(ma_);
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
  visualization_msgs::msg::MarkerArray ma;
  ma.markers.resize(1);
  ma.markers.front().action = visualization_msgs::msg::Marker::DELETEALL;
  markers_pub_->publish(ma);
  //  active_markers_.clear();
}

void Visualization::clearMarkers()
{
  ma_.markers.clear();
}

void Visualization::addMarkers(const std::vector<visualization_msgs::msg::Marker>& markers)
{
  ma_.markers.insert(ma_.markers.end(), markers.begin(), markers.end());
}

void Visualization::addTextMarker(const geometry_msgs::msg::PoseStamped& pose, const std::string& text, double size,
                                  const std_msgs::msg::ColorRGBA& color)
{
  ma_.markers.push_back(createTextMarker(pose, text, size, color));
}

void Visualization::addPointMarkers(const std::vector<geometry_msgs::msg::PoseStamped>& poses, double size,
                                    const std_msgs::msg::ColorRGBA& color)
{
  std::vector<visualization_msgs::msg::Marker> point_markers = createPointMarkers(poses, size, color);
  ma_.markers.insert(ma_.markers.end(), point_markers.begin(), point_markers.end());
}

void Visualization::addLineMarker(const std::vector<geometry_msgs::msg::PoseStamped>& poses, double size,
                                  const std_msgs::msg::ColorRGBA& color)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto& pose : poses)
  {
    points.push_back(pose.pose.position);
  }
  ma_.markers.push_back(createLineMarker(points, poses[0].header, size, color));
}

void Visualization::addBoxMarker(const geometry_msgs::msg::PoseStamped& pose, const std::array<double, 3>& dimensions,
                                 const std_msgs::msg::ColorRGBA& color)
{
  ma_.markers.push_back(createGeometryPrimitiveMarker(pose, dimensions, color, visualization_msgs::msg::Marker::CUBE));
}

void Visualization::addDiscMarker(const geometry_msgs::msg::PoseStamped& pose, double diameter,
                                  const std_msgs::msg::ColorRGBA& color)
{
  std::array<double, 3> dimensions{ diameter, diameter, 1e-6 };
  ma_.markers.push_back(createGeometryPrimitiveMarker(pose, dimensions, color, visualization_msgs::msg::Marker::CYLINDER));
}

visualization_msgs::msg::Marker Visualization::createMeshMarker(const geometry_msgs::msg::PoseStamped& pose,
                                                           const std::string& mesh_file,
                                                           const std::vector<double>& scale,
                                                           const std_msgs::msg::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_file;
  marker.ns = ns;
  marker.scale.x = (scale.size() > 0) ? scale[0] : 1.0;
  marker.scale.y = (scale.size() > 1) ? scale[1] : 1.0;
  marker.scale.z = (scale.size() > 2) ? scale[2] : 1.0;
  marker.color = color;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.header = pose.header;
  marker.pose = pose.pose;
  return marker;
}

visualization_msgs::msg::Marker Visualization::createGeometryPrimitiveMarker(const geometry_msgs::msg::PoseStamped& pose,
                                                                        const std::array<double, 3>& dimensions,
                                                                        const std_msgs::msg::ColorRGBA& color, int shape,
                                                                        const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.type = shape;
  marker.ns = ns;
  marker.scale.x = std::max(dimensions[0], MIN_SCALE);
  marker.scale.y = std::max(dimensions[1], MIN_SCALE);
  marker.scale.z = std::max(dimensions[2], MIN_SCALE);
  marker.color = color;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.header = pose.header;
  marker.pose = pose.pose;
  return marker;
}

visualization_msgs::msg::Marker Visualization::createLineMarker(const std::vector<geometry_msgs::msg::Point>& points,
                                                           const std_msgs::msg::Header& header, double length,
                                                           const std_msgs::msg::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.ns = ns;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = length;
  marker.color = color;
  marker.points = points;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.header = header;
  return marker;
}

visualization_msgs::msg::Marker Visualization::createPointMarker(const geometry_msgs::msg::PoseStamped& pose, double size,
                                                            const std_msgs::msg::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.header = pose.header;
  marker.pose = pose.pose;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color = color;
  marker.action = visualization_msgs::msg::Marker::ADD;
  return marker;
}

visualization_msgs::msg::Marker Visualization::createTextMarker(const geometry_msgs::msg::PoseStamped& poseSt,
                                                           const std::string& text, double size,
                                                           const std_msgs::msg::ColorRGBA& color, const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header = poseSt.header;
  marker.pose = poseSt.pose;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.ns = ns;
  marker.text = text;
  marker.color = color;
  marker.scale.z = size;
  marker.scale.x = size;  // RViz's space width; left at zero, a space is about 1 m wide
  return marker;
}

std::vector<visualization_msgs::msg::Marker>
Visualization::createPointMarkers(const std::vector<geometry_msgs::msg::PoseStamped>& poses, double size,
                                  const std_msgs::msg::ColorRGBA& color, const std::string& ns)
{
  std::vector<visualization_msgs::msg::Marker> markers;

  for (const auto& pose : poses)
  {
    visualization_msgs::msg::Marker marker;
    marker.ns = ns;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.header = pose.header;
    marker.pose = pose.pose;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color = color;
    marker.action = visualization_msgs::msg::Marker::ADD;
    markers.push_back(marker);
  }

  return markers;
}

visualization_msgs::msg::Marker Visualization::createCubeList(const geometry_msgs::msg::PoseStamped& ref_pose,
                                                         const std::vector<geometry_msgs::msg::Point>& poses,
                                                         const std::vector<std_msgs::msg::ColorRGBA>& colors, double size,
                                                         const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
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

visualization_msgs::msg::Marker Visualization::createPointList(const geometry_msgs::msg::PoseStamped& ref_pose,
                                                          const std::vector<geometry_msgs::msg::Point>& points,
                                                          const std::vector<std_msgs::msg::ColorRGBA>& colors, double size,
                                                          const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.header = ref_pose.header;
  marker.pose = ref_pose.pose;
  marker.ns = ns;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.points = points;
  marker.colors = colors;
  return marker;
}

rviz_2d_overlay_msgs::msg::OverlayText Visualization::createOverlayText(int offsetFromTop, const std_msgs::msg::ColorRGBA& color,
                                                               const std::string& text, double text_size)
{
  rviz_2d_overlay_msgs::msg::OverlayText msg;
  msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
  msg.width = 1000;
  msg.height = 25;
  msg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
  msg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
  msg.horizontal_distance = 5;
  msg.vertical_distance = offsetFromTop;
  msg.fg_color = color;
  msg.text = text;
  msg.text_size = text_size;
  return msg;
}

};  // namespace thorp::toolkit