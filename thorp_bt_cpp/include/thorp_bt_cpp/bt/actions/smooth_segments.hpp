#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "thorp_bt_cpp/bt/utils.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <mbf_msgs/ExePathResult.h>
#include <nav_msgs/Path.h>

#include <rr_nav_common/nav_graph.hpp>
#include <rr_nav_common/nav_utility.hpp>
#include <rr_nav_path_processor/path_processor.hpp>
#include "thorp_bt_cpp/route_progress_tracker.hpp"

namespace thorp::bt::actions
{
// TODO this can probably be refactored into more fine-grained actions
class SmoothSegments : public BT::SyncActionNode
{
  //! Footprint model types to use when checking path validity
  enum class FootprintModel
  {
    FOOTPRINT,
    INSCRIBED_RADIUS,
    CIRCUMSCRIBED_RADIUS
  };

public:
  SmoothSegments(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& pnh,
                 const common::graph::NavGraph& graph, const costmap_2d::Costmap2DROS& path_processor_costmap)
    : SyncActionNode(name, config)
    , pnh_(pnh)
    , graph_(graph)
    , path_processor_costmap_(path_processor_costmap)
    , path_processor_(path_processor_costmap.getCostmap())
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("holonomic_robot"),
             BT::InputPort<double>("min_turning_radius"),
             BT::InputPort<double>("max_turning_radius"),
             BT::InputPort<double>("progress_tracker_min_displacement"),
             BT::InputPort<double>("progress_tracker_max_displacement"),
             BT::InputPort<bool>("allow_undoing_progress"),
             BT::InputPort<unsigned long>("map_id"),
             BT::OutputPort<unsigned int>("error"),
             BT::OutputPort<std::vector<nav_msgs::Path>>("smooth_segments"),
             BT::BidirectionalPort<std::vector<Route>>("route_segments") };
  }

  BT::NodeStatus tick() override
  {
    namespace rrb = rapyuta::base;
    namespace rrnu = common::utility;

    const auto map_id = utils::getInput<unsigned long>(*this, pnh_, "map_id");
    const auto is_holonomic_robot = utils::getInput<bool>(*this, pnh_, "holonomic_robot");
    const auto max_turning_radius = utils::getInput<double>(*this, pnh_, "max_turning_radius");
    auto route_segments = utils::getInput<std::vector<Route>>(*this, pnh_, "route_segments");

    std::vector<nav_msgs::Path> smooth_segments;
    for (auto it = route_segments.begin(); it != route_segments.end(); ++it)
    {
      // Should never happen
      if (it->size() == 1)
      {
        ROS_ERROR_NAMED(name(), "Single checkpoint segment found");
        utils::setError(*this, mbf_msgs::ExePathResult::INTERNAL_ERROR);
        return BT::NodeStatus::FAILURE;
      }

      // convert data type
      nav_msgs::Path path = rrnu::routeToPath(*it);

      // no need to smooth
      if (it->size() == 2)
      {
        smooth_segments.push_back(path);
        continue;
      }

      // For holonomic robots on holonomic segments, we smooth and check the path assuming that the robot
      // is circular with inscribed radius, as we can arbitrarily rotate along the path to avoid obstacles
      const bool holonomic_segment = is_holonomic_robot && rrnu::isSegmentHolonomic(*it, graph_, map_id);
      const FootprintModel footprint_model =
          holonomic_segment ? FootprintModel::INSCRIBED_RADIUS : FootprintModel::FOOTPRINT;

      // Smooth the path for 5 iterations with max deviation from checkpoint limited by max_turning_radius
      if (!splineSmoothing(path, max_turning_radius, footprint_model))
      {
        ROS_ERROR_NAMED(name(), "Path smoothing failed, possibly due to collision");
        utils::setError(*this, mbf_msgs::ExePathResult::BLOCKED_PATH);
        return BT::NodeStatus::FAILURE;
      }

      // If path is split perform smoothing again
      if (splitSmoothPath(path, route_segments, it))
      {
        // TODO check feasibility according to the min_turning_radius since might require split again
        if (!splineSmoothing(path, max_turning_radius, footprint_model))
        {
          ROS_ERROR_NAMED(name(), "Path smoothing failed, possibly due to collision");
          utils::setError(*this, mbf_msgs::ExePathResult::BLOCKED_PATH);
          return BT::NodeStatus::FAILURE;
        }
      }

      smooth_segments.push_back(path);
    }

    setOutput("route_segments", route_segments);
    setOutput("smooth_segments", smooth_segments);
    return BT::NodeStatus::SUCCESS;
  }

private:
  /**
   * \brief Smoothen path by sampling intermediate points like a b-spline, simultaneously checking path validity
   * \param path       Path to smoothen and validate
   * \param max_radius Maximum turning radius allowed
   * \param footprint_model Which footprint model type to use when checking path validity (defaults to whole footprint)
   */
  bool splineSmoothing(nav_msgs::Path& path, double max_radius, FootprintModel footprint_model)
  {
    std::vector<geometry_msgs::Point> footprint = path_processor_costmap_.getRobotFootprint();
    if (footprint_model == FootprintModel::INSCRIBED_RADIUS || footprint_model == FootprintModel::CIRCUMSCRIBED_RADIUS)
    {
      double inscribed_radius, circumscribed_radius, used_radius;
      costmap_2d::calculateMinAndMaxDistances(footprint, inscribed_radius, circumscribed_radius);
      used_radius = footprint_model == FootprintModel::INSCRIBED_RADIUS ? inscribed_radius : circumscribed_radius;
      footprint = costmap_2d::makeFootprintFromRadius(used_radius);
    }

    path_processor_.updateFootprint(footprint);
    return path_processor_.smoothBSpline(path, 5, max_radius);
  }

  /**
   * @brief Split the smoothened path based on min_turning_radius
   * @param path smooth path to be processed
   * @param route_segments Vector of all path segments
   * @return true if path is split
   */
  bool splitSmoothPath(nav_msgs::Path& path, std::vector<rrnu::Route>& route_segments,
                       std::vector<rrnu::Route>::iterator& segment_it) const
  {
    namespace rrb = rapyuta::base;

    const auto min_turning_radius = utils::getInput<double>(*this, pnh_, "min_turning_radius");

    server::RouteProgressTracker lookup_table(utils::getInput<double>(*this, pnh_, "progress_tracker_min_displacement"),
                                              utils::getInput<double>(*this, pnh_, "progress_tracker_max_displacement"),
                                              utils::getInput<bool>(*this, pnh_, "allow_undoing_progress"));
    lookup_table.setCheckpoints(*segment_it);

    // TODO remove max_curvature and use curvature instead
    // no need to calculate max here and set to zero in the condition
    double max_curvature = 0;

    // TODO change to std::set since we need this unique and ordered
    std::vector<unsigned int> split_checkpoint_indices;

    // Iterate over poses to compute min turning radius and find indices to split at checkpoints with now turning radius
    for (auto pose_it = path.poses.cbegin(); pose_it != path.poses.cend() - 2; ++pose_it)
    {
      // we are using the progress tracker as a checkpoint lookup table; we need to "traverse" the
      // entire path virtually, so it reports the checkpoint associated with each pose correctly
      double progress_at_pose = lookup_table.getProgress(*(pose_it + 1));
      max_curvature = std::max(max_curvature, std::abs(rr::nav::path_processor::PathProcessor::curvature(
                                                  *pose_it, *(pose_it + 1), *(pose_it + 2))));
      const double turning_radius = 1.0 / max_curvature;

      if (turning_radius < min_turning_radius)
      {
        // Add the checkpoint index associated with the pose to the split vector
        split_checkpoint_indices.push_back(std::round(progress_at_pose));
        max_curvature = 0;
      }
    }

    if (!split_checkpoint_indices.empty())
    {
      std::vector<rrnu::Route> split_segments;

      // Split the current route segment
      if (!rrnu::splitRouteAtIndices(*segment_it, split_checkpoint_indices, split_segments))
      {
        ROS_WARN_NAMED(name(), "Failed to split segment to honor min turning radius; proceeding anyway");
        return false;
      }

      ROS_INFO_NAMED(name(), "Split current segment into %lu segments due to lower than %.2f turning radius",
                     split_segments.size(), min_turning_radius);

      // Remove current segment from the segment list
      segment_it = route_segments.erase(segment_it);

      // Insert split segments
      segment_it = route_segments.insert(segment_it, std::make_move_iterator(split_segments.begin()),
                                         std::make_move_iterator(split_segments.end()));
      // Update nav path to contain current split segment poses
      path = rrnu::routeToPath(route_segments.at(std::distance(route_segments.begin(), segment_it)));
      return true;
    }
    else
    {
      ROS_INFO_NAMED(name(), "Min turning radius of smoothened path: %.2fm", 1.0 / max_curvature);
    }

    // return false as no splitting required
    return false;
  }

private:
  ros::NodeHandle pnh_;
  const common::graph::NavGraph& graph_;
  const costmap_2d::Costmap2DROS& path_processor_costmap_;
  rr::nav::path_processor::PathProcessor path_processor_;
};
}  // namespace thorp::bt::actions
