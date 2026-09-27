/*
 * Template matching; see template_matcher.hpp for the code origin.
 */

#include "thorp_perception/template_matcher.hpp"

#include <filesystem>
#include <limits>
#include <stdexcept>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_2D.h>

namespace thorp::perception
{
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

TemplateMatcher::TemplateMatcher(const std::string& templates_path, const IcpParams& params) : params_(params)
{
  for (const auto& file : std::filesystem::directory_iterator(templates_path))
  {
    if (file.path().extension() != ".pcd")
      continue;
    auto cloud = std::make_shared<PointCloudXYZ>();
    if (pcl::io::loadPCDFile(file.path().string(), *cloud) < 0)
      throw std::runtime_error("Failed to load template " + file.path().string());
    templates_.emplace_back(file.path().stem().string(), cloud);
  }
}

TemplateMatch TemplateMatcher::match(const PointCloudXYZ::ConstPtr& cluster,
                                     const Eigen::Isometry3f& initial_estimate) const
{
  std::vector<TemplateMatch> matches(templates_.size());
#pragma omp parallel for
  for (size_t i = 0; i < templates_.size(); ++i)
  {
    auto template_cloud = std::make_shared<PointCloudXYZ>();
    pcl::transformPointCloud(*templates_[i].second, *template_cloud, initial_estimate.matrix());

    // We align the cluster to the template, restricted to translations and rotations on the xy plane
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cluster);
    icp.setInputTarget(template_cloud);
    icp.setMaximumIterations(params_.iterations);
    icp.setMaxCorrespondenceDistance(params_.max_distance);
    icp.setTransformationEpsilon(params_.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(params_.fitness_epsilon);
    icp.setRANSACOutlierRejectionThreshold(params_.max_distance / 2.0);
    icp.setTransformationEstimation(
        std::make_shared<pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>>());
    PointCloudXYZ aligned;
    bool aligned_ok = false;
    try
    {
      icp.align(aligned);
      aligned_ok = icp.hasConverged();
    }
    catch (const std::exception&)
    {
      // Degenerate clouds can make the alignment fail; exceptions can't leave an OpenMP loop
    }

    matches[i].name = templates_[i].first;
    if (aligned_ok)
    {
      // The template is where the inverse of the cluster alignment takes it
      Eigen::Isometry3f alignment(icp.getFinalTransformation());
      matches[i].pose = alignment.inverse() * initial_estimate;
      matches[i].error = icp.getFitnessScore();
    }
    else
    {
      matches[i].error = std::numeric_limits<double>::infinity();
    }
  }

  TemplateMatch best{ "", initial_estimate, std::numeric_limits<double>::infinity() };
  for (const auto& match : matches)
    if (match.error < best.error)
      best = match;
  return best;
}

}  // namespace thorp::perception
