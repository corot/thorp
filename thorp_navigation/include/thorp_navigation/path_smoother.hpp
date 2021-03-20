/**
 *  \file       path_smoother.hpp
 *  \brief      Class for smoothing paths using bspline
 */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>

/**
 * This class is responsible for creating smooth paths under these assumptions:
 * - The path may not pass through all checkpoints
 * - Input path states are valid w.r.t. the costmap
 */
class PathSmoother
{
public:
  PathSmoother();

  /**
   * \brief Smoothen path by sampling intermediate points like a bspline
   * \param path       Path to be smoothened
   * \param max_steps  Number of sampling iterations
   * \param max_radius Maximum turning radius allowed
   */
  bool smoothBSpline(nav_msgs::Path& path, double max_steps = 5, double max_radius = 5.);

  /**
   * \brief Publish discretized poses as markers
   * \param poses      Poses to be visualized
   * \param ns         Marker namespace
   * \param red        Marker red value
   * \param green      Marker green value
   * \param blue       Marker blue value
   */
  void publishPoses(const std::vector<geometry_msgs::PoseStamped>& poses, std::string ns,
                    double red, double green, double blue);

  /**
   * \brief Insert points between every pair of points on path until the distance between two points is less the
   * resolution
   * \param path       Path to be subdivided
   * \param resolution Discretization resolution
   */
  static bool rediscretize(nav_msgs::Path& path, double resolution);

  /**
   * \brief Weighted linear interpolation between two points. The interpolated pose's orientation will be the heading
   * from start to end poses (or end to start if we are moving backwards).
   * \param start      Initial point
   * \param end        Final point
   * \param weight     Ratio of interpolation weight (range [0 1]) with 0 meaning end and 1 meaning start
   * \param result     Interpolated point
   */
  static bool interpolate(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end, double weight,
                          geometry_msgs::PoseStamped& result);

  bool connectWaypointsSrv(thorp_msgs::ConnectWaypoints::Request& req, thorp_msgs::ConnectWaypoints::Response& res);

private:
  /**
   * \brief Check if a point lies in free space on the costmap
   * \param pose       Point at which state validity is checked
   * \param highlight_invalid_pose Highlight the invalid state with a visual marker and log a warn
   */
  bool isStateValid(const geometry_msgs::PoseStamped& pose, bool highlight_invalid_pose = false);

  /**
   * \brief Check if all points between two points after linear interpolation are valid
   * \param start      Start point for interpolation
   * \param end        End point for interpolation
   * \param resolution Distance between each pair of interpolated points (in m)
   * \param highlight_invalid_pose Highlight the invalid state with a visual marker and log a warn
   */
  bool isMotionValid(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
                     double step_size = 0.05, bool highlight_invalid_pose = false);

  /**
   * \brief Insert mid points between every pair of points on path
   * \param path       Path to be subdivided
   */
  bool subdivide(nav_msgs::Path& path);

  /**
   * \brief Insert pose if valid after doing weighted linear interpolation between two points
   * \param start      Initial point
   * \param end        Final point
   * \param weight     Ratio of interpolation weight (range [0 1]) with 0 meaning start and 1 meaning end
   * \param pose_list  Pose vector to insert into
   */
  bool insertInterpolatedIfValid(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
                                 double weight, std::vector<geometry_msgs::PoseStamped>& pose_list);
  /**
   * \brief Insert points between every pair of points on path at a given distance
   * \param path       Path to be subdivided
   * \param distance   Points generated at this distance from vertices of each segment
   */
  bool insertAtDistance(nav_msgs::Path& path, double distance);

  /**
   * \brief Highlight a given pose to allow easy debugging
   * @param pose       Pose to highlight
   */
  void highlightPose(const geometry_msgs::PoseStamped& pose) const;

  /**
   * \brief Compute area enclosed by the triangle
   * \param point_a    First point
   * \param point_b    Second point
   * \param point_c    Third point
   */
  static double enclosedArea(const geometry_msgs::PoseStamped& point_a,
                             const geometry_msgs::PoseStamped& point_b,
                             const geometry_msgs::PoseStamped& point_c);
  /**
   * \brief Compute Menger curvature between three points
   * \param point_a    First point
   * \param point_b    Second point
   * \param point_c    Third point
   */
  static double curvature(const geometry_msgs::PoseStamped& point_a, const geometry_msgs::PoseStamped& point_b,
                          const geometry_msgs::PoseStamped& point_c);

  /// Allow smoothed path to traverse unknown space
  bool allow_unknown_space_;

  /// Costmap2DROS object
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_2d_;

  /// Costmap model for footprint collision checking
  std::shared_ptr<base_local_planner::CostmapModel> costmap_model_;

  ros::NodeHandle pnh_;

  ros::Publisher viz_result_pub_;

  ros::ServiceServer connect_wp_srv_;
};