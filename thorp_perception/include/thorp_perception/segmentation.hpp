/*
 * Tabletop segmentation: find a horizontal support surface and the object clusters above it.
 *
 * The surface search and the table orientation from the largest quadrilateral inscribed in the surface's convex hull
 * come from corot/rail_segmentation (a fork of GT-RAIL/rail_segmentation, BSD license, Copyright (c) 2015, Worcester
 * Polytechnic Institute); see the package README.
 */

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace thorp::perception
{
using PointT = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<PointT>;

struct SegmentationParams
{
  // Region to segment, on the cloud frame
  Eigen::Vector3f crop_min{ 0.15f, -1.0f, 0.3f };
  Eigen::Vector3f crop_max{ 2.5f, 1.0f, 0.6f };
  int min_surface_size = 10000;  // points
  int min_cluster_size = 200;    // points
  double cluster_tolerance = 0.005;
};

/**
 * Horizontal support surface, as the largest quadrilateral inscribed in its convex hull.
 */
struct Surface
{
  PointCloud::Ptr cloud;
  Eigen::Vector3f centroid;    // of the surface points
  Eigen::Vector2f center;      // of the quadrilateral
  double yaw;                  // of the quadrilateral's longest side
  double length;               // longest side
  double width;                // longer of the two shortest sides
  std::vector<Eigen::Vector2f> corners;
};

/**
 * Remove NaN points and those out of the segmentation region.
 */
PointCloud::Ptr crop(const PointCloud& cloud, const SegmentationParams& params);

/**
 * Find the largest horizontal surface in the cloud.
 * @param cloud Cropped cloud, on a frame with z axis up
 * @param params Segmentation parameters
 * @param min_side Minimum length of the surface's shortest side
 * @param error Reason for not finding a surface
 * @return The surface, or nothing if there's no surface of the minimum size and side
 */
std::optional<Surface> findSurface(const PointCloud::ConstPtr& cloud, const SegmentationParams& params,
                                   double min_side, std::string& error);

/**
 * Extract the clusters of points above a surface, as separate clouds.
 */
std::vector<PointCloud::Ptr> extractClusters(const PointCloud::ConstPtr& cloud, const Surface& surface,
                                             const SegmentationParams& params);

/**
 * The largest quadrilateral with corners on the given convex polygon vertices, in polygon order.
 */
std::vector<Eigen::Vector2f> largestQuadrilateral(const std::vector<Eigen::Vector2f>& polygon);

}  // namespace thorp::perception
