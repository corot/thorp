#pragma once

#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "thorp_toolkit/common.hpp"

namespace thorp::toolkit
{

class Visualization
{
public:
  static constexpr double MIN_SCALE = 1e-6;

  static visualization_msgs::msg::Marker createMeshMarker(const geometry_msgs::msg::PoseStamped& pose,
                                                     const std::string& mesh_file, const std::vector<double>& scale,
                                                     const std_msgs::msg::ColorRGBA& color = namedColor("blue"),
                                                     const std::string& ns = "mesh_marker");
  static visualization_msgs::msg::Marker createGeometryPrimitiveMarker(const geometry_msgs::msg::PoseStamped& pose,
                                                                  const std::array<double, 3>& dimensions,
                                                                  const std_msgs::msg::ColorRGBA& color = namedColor("blue"),
                                                                  int shape = visualization_msgs::msg::Marker::CUBE,
                                                                  const std::string& ns = "gp_marker");
  static visualization_msgs::msg::Marker createLineMarker(const std::vector<geometry_msgs::msg::Point>& points,
                                                     const std_msgs::msg::Header& header, double length = 0.01,
                                                     const std_msgs::msg::ColorRGBA& color = namedColor("blue"),
                                                     const std::string& ns = "line_strip");
  static visualization_msgs::msg::Marker createPointMarker(const geometry_msgs::msg::PoseStamped& pose, double size = 0.01,
                                                      const std_msgs::msg::ColorRGBA& color = namedColor("blue"),
                                                      const std::string& ns = "point_marker");
  static visualization_msgs::msg::Marker createTextMarker(const geometry_msgs::msg::PoseStamped& poseSt, const std::string& text,
                                                     double size = 0.1,
                                                     const std_msgs::msg::ColorRGBA& color = namedColor("blue"),
                                                     const std::string& ns = "text_marker");
  static std::vector<visualization_msgs::msg::Marker>
  createPointMarkers(const std::vector<geometry_msgs::msg::PoseStamped>& poses, double size = 0.01,
                     const std_msgs::msg::ColorRGBA& color = namedColor("blue"), const std::string& ns = "point_marker");
  static visualization_msgs::msg::Marker createCubeList(const geometry_msgs::msg::PoseStamped& ref_pose,
                                                   const std::vector<geometry_msgs::msg::Point>& poses,
                                                   const std::vector<std_msgs::msg::ColorRGBA>& colors, double size = 0.01,
                                                   const std::string& ns = "cube_list");
  static visualization_msgs::msg::Marker createPointList(const geometry_msgs::msg::PoseStamped& ref_pose,
                                                    const std::vector<geometry_msgs::msg::Point>& points,
                                                    const std::vector<std_msgs::msg::ColorRGBA>& colors, double size = 0.01,
                                                    const std::string& ns = "point_list");
  static rviz_2d_overlay_msgs::msg::OverlayText createOverlayText(int offsetFromTop, const std_msgs::msg::ColorRGBA& color,
                                                         const std::string& text, double text_size);

  Visualization(const std::string& topic = "visual_markers", double lifetime = 60);
  void publishMarkers(int start_id = 1);
  void reset();
  void deleteMarkers();
  void clearMarkers();
  void addMarkers(const std::vector<visualization_msgs::msg::Marker>& markers);
  void addTextMarker(const geometry_msgs::msg::PoseStamped& pose, const std::string& text, double size = 0.02,
                     const std_msgs::msg::ColorRGBA& color = namedColor("blue"));
  void addPointMarkers(const std::vector<geometry_msgs::msg::PoseStamped>& poses, double size = 0.02,
                       const std_msgs::msg::ColorRGBA& color = namedColor("blue"));
  void addLineMarker(const std::vector<geometry_msgs::msg::PoseStamped>& poses, double length,
                     const std_msgs::msg::ColorRGBA& color = namedColor("blue"));
  void addBoxMarker(const geometry_msgs::msg::PoseStamped& pose, const std::array<double, 3>& dimensions,
                    const std_msgs::msg::ColorRGBA& color = namedColor("blue"));
  void addDiscMarker(const geometry_msgs::msg::PoseStamped& pose, double diameter,
                     const std_msgs::msg::ColorRGBA& color = namedColor("blue"));

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  double lifetime_;
  visualization_msgs::msg::MarkerArray ma_;
  //  std::vector<visualization_msgs::msg::Marker> active_markers_;  // published markers
};

} /* namespace thorp::toolkit */
