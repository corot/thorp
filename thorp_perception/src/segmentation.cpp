/*
 * Tabletop segmentation; see segmentation.hpp for the code origin.
 */

#include "thorp_perception/segmentation.hpp"

#include <algorithm>
#include <cmath>

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

namespace thorp::perception
{
namespace
{
// Plane fitting: tolerance to horizontal, and inliers distance to the plane
constexpr double SAC_EPS_ANGLE = 0.15;
constexpr double SAC_DISTANCE_THRESHOLD = 0.01;
constexpr int SAC_MAX_ITERATIONS = 100;
// Objects must be above the surface by this margin, to not include surface points
constexpr double SURFACE_REMOVAL_PADDING = 0.002;
}  // namespace

PointCloud::Ptr crop(const PointCloud& cloud, const SegmentationParams& params)
{
  // CropBox discards NaN points too
  pcl::CropBox<PointT> crop_box;
  crop_box.setInputCloud(cloud.makeShared());
  crop_box.setMin(Eigen::Vector4f(params.crop_min.x(), params.crop_min.y(), params.crop_min.z(), 1.0f));
  crop_box.setMax(Eigen::Vector4f(params.crop_max.x(), params.crop_max.y(), params.crop_max.z(), 1.0f));
  auto cropped = std::make_shared<PointCloud>();
  crop_box.filter(*cropped);
  return cropped;
}

std::optional<Surface> findSurface(const PointCloud::ConstPtr& cloud, const SegmentationParams& params,
                                   double min_side, std::string& error)
{
  if (static_cast<int>(cloud->size()) < params.min_surface_size)
  {
    error = "cropped cloud too small to contain a surface (" + std::to_string(cloud->size()) + " points)";
    return std::nullopt;
  }

  // Largest horizontal plane, using the normals to reject points of vertical faces at the plane height
  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimation;
  normal_estimation.setSearchMethod(std::make_shared<pcl::search::KdTree<PointT>>());
  normal_estimation.setKSearch(50);
  normal_estimation.setInputCloud(cloud);
  normal_estimation.compute(*normals);

  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> plane_segmentation;
  plane_segmentation.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
  plane_segmentation.setMethodType(pcl::SAC_RANSAC);
  plane_segmentation.setNormalDistanceWeight(0.1);
  plane_segmentation.setAxis(Eigen::Vector3f::UnitZ());
  plane_segmentation.setEpsAngle(SAC_EPS_ANGLE);
  plane_segmentation.setDistanceThreshold(SAC_DISTANCE_THRESHOLD);
  plane_segmentation.setMaxIterations(SAC_MAX_ITERATIONS);
  plane_segmentation.setProbability(0.99);
  plane_segmentation.setInputCloud(cloud);
  plane_segmentation.setInputNormals(normals);
  auto inliers = std::make_shared<pcl::PointIndices>();
  pcl::ModelCoefficients coefficients;
  plane_segmentation.segment(*inliers, coefficients);
  if (static_cast<int>(inliers->indices.size()) < params.min_surface_size)
  {
    error = "no surface with more than " + std::to_string(params.min_surface_size) + " points (largest has " +
            std::to_string(inliers->indices.size()) + ")";
    return std::nullopt;
  }

  Surface surface;
  surface.cloud = std::make_shared<PointCloud>();
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.filter(*surface.cloud);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*surface.cloud, centroid);
  surface.centroid = centroid.head<3>();

  // Table shape: the largest quadrilateral inscribed in the surface points' convex hull, on the xy plane
  auto projected = std::make_shared<PointCloud>(*surface.cloud);
  for (auto& point : projected->points)
    point.z = 0.0f;
  PointCloud hull;
  pcl::ConvexHull<PointT> convex_hull;
  convex_hull.setDimension(2);
  convex_hull.setInputCloud(projected);
  convex_hull.reconstruct(hull);
  if (hull.size() < 4)
  {
    error = "surface convex hull has less than four points (" + std::to_string(hull.size()) + ")";
    return std::nullopt;
  }
  std::vector<Eigen::Vector2f> polygon;
  for (const auto& point : hull.points)
    polygon.emplace_back(point.x, point.y);
  surface.corners = largestQuadrilateral(polygon);

  // Sides sorted from longest to shortest; the shortest must reach the minimum side
  std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> sides;
  for (size_t i = 0; i < 4; ++i)
    sides.emplace_back(surface.corners[i], surface.corners[(i + 1) % 4]);
  std::sort(sides.begin(), sides.end(), [](const auto& a, const auto& b) {
    return (a.second - a.first).norm() > (b.second - b.first).norm();
  });
  if ((sides[3].second - sides[3].first).norm() < min_side)
  {
    error = "surface shortest side below the minimum of " + std::to_string(min_side) + " m";
    return std::nullopt;
  }
  Eigen::Vector2f longest = sides[0].second - sides[0].first;
  surface.yaw = std::atan2(longest.y(), longest.x());
  surface.length = longest.norm();
  surface.width = (sides[2].second - sides[2].first).norm();
  surface.center = (surface.corners[0] + surface.corners[1] + surface.corners[2] + surface.corners[3]) / 4.0f;
  return surface;
}

std::vector<PointCloud::Ptr> extractClusters(const PointCloud::ConstPtr& cloud, const Surface& surface,
                                             const SegmentationParams& params)
{
  auto above_surface = std::make_shared<pcl::Indices>();
  for (size_t i = 0; i < cloud->size(); ++i)
    if ((*cloud)[i].z > surface.centroid.z() + SURFACE_REMOVAL_PADDING)
      above_surface->push_back(static_cast<int>(i));

  std::vector<pcl::PointIndices> clusters_indices;
  pcl::EuclideanClusterExtraction<PointT> clustering;
  auto kd_tree = std::make_shared<pcl::search::KdTree<PointT>>();
  kd_tree->setInputCloud(cloud, above_surface);
  clustering.setClusterTolerance(params.cluster_tolerance);
  clustering.setMinClusterSize(params.min_cluster_size);
  clustering.setSearchMethod(kd_tree);
  clustering.setInputCloud(cloud);
  clustering.setIndices(above_surface);
  clustering.extract(clusters_indices);

  std::vector<PointCloud::Ptr> clusters;
  for (const auto& indices : clusters_indices)
    clusters.push_back(std::make_shared<PointCloud>(*cloud, indices.indices));
  return clusters;
}

std::vector<Eigen::Vector2f> largestQuadrilateral(const std::vector<Eigen::Vector2f>& polygon)
{
  // Shoelace formula; the corners are in polygon order, so the quadrilateral is convex
  auto area = [](const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, const Eigen::Vector2f& p3,
                 const Eigen::Vector2f& p4) {
    return std::abs(p1.x() * p2.y() + p2.x() * p3.y() + p3.x() * p4.y() + p4.x() * p1.y() - p1.y() * p2.x() -
                    p2.y() * p3.x() - p3.y() * p4.x() - p4.y() * p1.x()) /
           2.0;
  };

  double max_area = -1.0;
  std::vector<Eigen::Vector2f> largest(4);
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i)
    for (size_t j = i + 1; j < n; ++j)
      for (size_t k = j + 1; k < n; ++k)
        for (size_t l = k + 1; l < n; ++l)
        {
          double quad_area = area(polygon[i], polygon[j], polygon[k], polygon[l]);
          if (quad_area > max_area)
          {
            max_area = quad_area;
            largest = { polygon[i], polygon[j], polygon[k], polygon[l] };
          }
        }
  return largest;
}

}  // namespace thorp::perception
