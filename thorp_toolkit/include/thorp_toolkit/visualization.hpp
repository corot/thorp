#pragma once

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "thorp_toolkit/common.hpp"
// #include "singleton.h"  // Include the header file for Singleton

namespace thorp::toolkit
{

class Visualization
{  // : public Singleton<Visualization> {
public:
  static constexpr double MIN_SCALE = 1e-6;

  static visualization_msgs::Marker createMeshMarker(const geometry_msgs::PoseStamped& pose,
                                                     const std::string& mesh_file, const std::vector<double>& scale,
                                                     const std_msgs::ColorRGBA& color = namedColor("blue"),
                                                     const std::string& ns = "mesh_marker");
  static visualization_msgs::Marker createGeometryPrimitiveMarker(const geometry_msgs::PoseStamped& pose,
                                                                  const std::array<double, 3>& dimensions,
                                                                  const std_msgs::ColorRGBA& color = namedColor("blue"),
                                                                  int shape = visualization_msgs::Marker::CUBE,
                                                                  const std::string& ns = "gp_marker");
  static visualization_msgs::Marker createLineMarker(const std::vector<geometry_msgs::Point>& points,
                                                     const std_msgs::Header& header, double length = 0.01,
                                                     const std_msgs::ColorRGBA& color = namedColor("blue"),
                                                     const std::string& ns = "line_strip");
  static visualization_msgs::Marker createPointMarker(const geometry_msgs::PoseStamped& pose, double size = 0.01,
                                                      const std_msgs::ColorRGBA& color = namedColor("blue"),
                                                      const std::string& ns = "point_marker");
  static visualization_msgs::Marker createTextMarker(const geometry_msgs::PoseStamped& poseSt, const std::string& text,
                                                     double size = 0.1,
                                                     const std_msgs::ColorRGBA& color = namedColor("blue"),
                                                     const std::string& ns = "text_marker");
  static std::vector<visualization_msgs::Marker>
  createPointMarkers(const std::vector<geometry_msgs::PoseStamped>& poses, double size = 0.01,
                     const std_msgs::ColorRGBA& color = namedColor("blue"), const std::string& ns = "point_marker");
  static visualization_msgs::Marker createCubeList(const geometry_msgs::PoseStamped& ref_pose,
                                                   const std::vector<geometry_msgs::Point>& poses,
                                                   const std::vector<std_msgs::ColorRGBA>& colors, double size = 0.01,
                                                   const std::string& ns = "cube_list");
  static visualization_msgs::Marker createPointList(const geometry_msgs::PoseStamped& ref_pose,
                                                    const std::vector<geometry_msgs::Point>& points,
                                                    const std::vector<std_msgs::ColorRGBA>& colors, double size = 0.01,
                                                    const std::string& ns = "point_list");
  static jsk_rviz_plugins::OverlayText createOverlayText(int offsetFromTop, const std_msgs::ColorRGBA& color,
                                                         const std::string& text, double text_size);

  Visualization(const std::string& topic = "visual_markers", double lifetime = 60);
  void publishMarkers(int start_id = 1);
  void reset();
  void deleteMarkers();
  void clearMarkers();
  void addMarkers(const std::vector<visualization_msgs::Marker>& markers);
  void addTextMarker(const geometry_msgs::PoseStamped& pose, const std::string& text, double size = 0.02,
                     const std_msgs::ColorRGBA& color = namedColor("blue"));
  void addPointMarkers(const std::vector<geometry_msgs::PoseStamped>& poses, double size = 0.02,
                       const std_msgs::ColorRGBA& color = namedColor("blue"));
  void addLineMarker(const std::vector<geometry_msgs::PoseStamped>& poses, double length,
                     const std_msgs::ColorRGBA& color = namedColor("blue"));
  void addBoxMarker(const geometry_msgs::PoseStamped& pose, const std::array<double, 3>& dimensions,
                    const std_msgs::ColorRGBA& color = namedColor("blue"));
  void addDiscMarker(const geometry_msgs::PoseStamped& pose, double diameter,
                     const std_msgs::ColorRGBA& color = namedColor("blue"));

private:
  ros::Publisher markers_pub_;
  double lifetime_;
  visualization_msgs::MarkerArray ma_;
  //  std::vector<visualization_msgs::Marker> active_markers_;  // published markers
};

} /* namespace thorp::toolkit */
