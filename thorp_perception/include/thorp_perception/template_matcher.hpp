/*
 * Identify object clusters by matching them against templates with a 2D ICP, as tabletop objects can only turn around
 * the vertical axis.
 *
 * The matching comes from corot/rail_mesh_icp (a fork of GT-RAIL/rail_mesh_icp, BSD license, Copyright (c) 2019,
 * Robot Autonomy and Interactive Learning); see the package README.
 */

#pragma once

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace thorp::perception
{
struct IcpParams
{
  int iterations = 50;
  double max_distance = 0.01;
  double transformation_epsilon = 1e-8;
  double fitness_epsilon = 1e-8;
};

struct TemplateMatch
{
  std::string name;
  Eigen::Isometry3f pose;  // of the template's origin, at its base
  double error;            // ICP fitness score
};

class TemplateMatcher
{
public:
  /**
   * @param templates_path Directory with the templates, as PCD files named after the object types
   */
  TemplateMatcher(const std::string& templates_path, const IcpParams& params);

  size_t size() const
  {
    return templates_.size();
  }

  /**
   * Match a cluster against all the templates.
   * @param cluster Object points
   * @param initial_estimate Initial pose for the templates
   * @return The best match, or a match with infinite error if all failed
   */
  TemplateMatch match(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cluster,
                      const Eigen::Isometry3f& initial_estimate) const;

private:
  IcpParams params_;
  std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>> templates_;
};

}  // namespace thorp::perception
