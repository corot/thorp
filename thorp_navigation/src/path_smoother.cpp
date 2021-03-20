#include <tf/tf.h>

#include <visualization_msgs/MarkerArray.h>

#include <thorp_msgs/ConnectWaypoints.h>
#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_navigation/path_smoother.hpp"

PathSmoother::PathSmoother() : pnh_("~")
{
  allow_unknown_space_ = pnh_.param("allow_unknown_space", false);
  costmap_2d_ = std::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("path_smoother_costmap",
                                                                                       ttk::TF2::instance().buffer()));
  costmap_2d_->start();
  costmap_model_ = std::shared_ptr<base_local_planner::CostmapModel>(
      new base_local_planner::CostmapModel(*costmap_2d_->getCostmap()));
  viz_result_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("smoothed_path", 1, true);
  connect_wp_srv_ = pnh_.advertiseService("connect_waypoints", &PathSmoother::connectWaypointsSrv, this);
  ROS_INFO("Path smoother server ready");
}

bool PathSmoother::subdivide(nav_msgs::Path& path)
{
  if (path.poses.size() < 2)
  {
    ROS_ERROR("At least 2 points in path is required");
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> new_states;
  new_states.reserve(floor(3 * path.poses.size() / 2));
  new_states.push_back(path.poses[0]);
  for (unsigned int i = 1; i < path.poses.size(); ++i)
  {
    if (!insertInterpolatedIfValid(path.poses.at(i - 1), path.poses.at(i), 0.5, new_states))
      return false;
    new_states.push_back(path.poses.at(i));
  }
  path.poses = std::move(new_states);
  return true;
}

bool PathSmoother::rediscretize(nav_msgs::Path& path, double resolution)
{
  if (path.poses.size() < 2)
  {
    ROS_ERROR("At least 2 points in path is required");
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> new_states, temp_states;
  new_states.reserve(floor(3 * path.poses.size() / 2));
  new_states.push_back(path.poses[0]);
  for (unsigned int i = 1; i < path.poses.size(); ++i)
  {
    double length = ttk::distance2D(new_states.back(), path.poses.at(i));
    double step_size = resolution / length;
    for (double weight = step_size; weight < 1; weight += step_size)
    {
      geometry_msgs::PoseStamped temp_pose;
      interpolate(new_states.back(), path.poses.at(i), weight, temp_pose);
      temp_states.push_back(temp_pose);
    }
    new_states.insert(new_states.end(), temp_states.begin(), temp_states.end());
    temp_states.clear();
  }
  new_states.push_back(path.poses.back());
  path.poses = std::move(new_states);
  return true;
}

bool PathSmoother::insertAtDistance(nav_msgs::Path& path, double distance)
{
  if (path.poses.size() < 2)
  {
    ROS_ERROR("At least 2 points in path is required");
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> new_states;
  new_states.reserve(floor(3 * path.poses.size() / 2));
  new_states.push_back(path.poses[0]);
  for (unsigned int i = 1; i < path.poses.size(); ++i)
  {
    double length = ttk::distance2D(new_states.back(), path.poses.at(i));
    if (length > 2 * distance)
    {
      if (!insertInterpolatedIfValid(path.poses.at(i - 1), path.poses.at(i), distance / length, new_states))
        return false;
      if (!insertInterpolatedIfValid(path.poses.at(i - 1), path.poses.at(i), 1 - distance / length, new_states))
        return false;
    }
    else if (!insertInterpolatedIfValid(path.poses.at(i - 1), path.poses.at(i), 0.5, new_states))
      return false;
    new_states.push_back(path.poses.at(i));
  }
  path.poses = std::move(new_states);
  return true;
}

bool PathSmoother::smoothBSpline(nav_msgs::Path& path, double max_steps, double max_radius)
{
  double resolution = costmap_2d_->getCostmap()->getResolution();
  for (unsigned int idx = 1; idx < path.poses.size(); ++idx)
  {
    if (!isMotionValid(path.poses.at(idx - 1), path.poses.at(idx), resolution, true))
    {
      ROS_ERROR("Input path is in collision, can't perform smoothing");
      return false;
    }
  }

  double max_corner_deviation = 2 * (2 - sqrt(2)) * max_radius;  // distance from corner to the inscribed arc
  if (!insertAtDistance(path, max_corner_deviation))
    return false;
  geometry_msgs::PoseStamped temp_pose_1, temp_pose_2;
  for (unsigned int step = 0; step < max_steps; ++step)
  {
    if (!subdivide(path))
      return false;
    unsigned int num_inserted_poses = 0;
    for (unsigned int i = 2; i < path.poses.size() - 1; i += 2)
    {
      if (isMotionValid(path.poses.at(i - 1), path.poses.at(i), resolution) &&
          isMotionValid(path.poses.at(i), path.poses.at(i + 1), resolution))  // Check motion bw path.poses.at(i - 1)
                                                                              // and path.poses[i - 1]for validity
      {
        interpolate(path.poses.at(i - 1), path.poses.at(i), 0.5, temp_pose_1);
        interpolate(path.poses.at(i), path.poses.at(i + 1), 0.5, temp_pose_2);
        interpolate(temp_pose_1, temp_pose_2, 0.5, temp_pose_1);
        if (isMotionValid(path.poses.at(i - 1), temp_pose_1, resolution) &&
            isMotionValid(temp_pose_1, path.poses.at(i + 1), resolution))  // Check motion bw path.poses[i-1] and
                                                                           // temp_pose_1, temp_pose_1 and
                                                                           // path.poses[i+1] for validity
        {
          if (ttk::distance2D(path.poses.at(i), temp_pose_1) > 0)  // Insert temp_pose_1 only if path.poses.at(i) and
                                                                   // temp_pose_1 aren't too close
          {
            path.poses.at(i) = temp_pose_1;
            ++num_inserted_poses;
          }
        }
      }
    }
    if (num_inserted_poses == 0)
      break;
  }
  return true;
}

bool PathSmoother::isStateValid(const geometry_msgs::PoseStamped& pose, bool highlight_invalid_pose)
{
  // Transform footprint to the pose
  std::vector<geometry_msgs::Point> oriented_footprint;
  costmap_2d::transformFootprint(pose.pose.position.x, pose.pose.position.y, ttk::yaw(pose),
      costmap_2d_->getRobotFootprint(), oriented_footprint);
  // Evaluate cost of the footprint
  double footprint_cost = costmap_model_->footprintCost(pose.pose.position, oriented_footprint, 0.0, 0.0);
  bool valid = footprint_cost >= 0 || (allow_unknown_space_ && footprint_cost == -2.0);
  if (!valid && highlight_invalid_pose)
  {
    // Highlight the problematic pose to allow easy debugging
    ROS_WARN("Invalid state at %s; cost: %g", ttk::pose2cstr2D(pose), footprint_cost);
    highlightPose(pose);
  }
  return valid;
}

bool PathSmoother::isMotionValid(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
                                 double resolution, bool highlight_invalid_pose)
{
  double length = ttk::distance2D(start, end);
  double step_size = resolution / length;
  bool is_valid = true;
  geometry_msgs::PoseStamped interpolated_point;
  for (double weight = 0; weight < 1 && is_valid; weight += step_size)
  {
    interpolate(start, end, weight, interpolated_point);
    is_valid &= isStateValid(interpolated_point, highlight_invalid_pose);
  }
  // For end pose when weight = 1
  interpolate(start, end, 1.0, interpolated_point);
  is_valid &= isStateValid(interpolated_point, highlight_invalid_pose);
  return is_valid;
}

bool PathSmoother::interpolate(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
                               double weight, geometry_msgs::PoseStamped& result)
{
  if (weight > 1 || weight < 0)
  {
    ROS_ERROR("Weights should be within [0 1] range");
    return false;
  }
  result.pose.position.x = (1 - weight) * start.pose.position.x + weight * end.pose.position.x;
  result.pose.position.y = (1 - weight) * start.pose.position.y + weight * end.pose.position.y;

  // orientation: heading from start to end poses (or end to start if we are moving backwards)
  double pose_to_pose_yaw = ttk::heading(start, end);
  bool forward = std::abs(ttk::normAngle(pose_to_pose_yaw - ttk::yaw(start))) < M_PI_2;
  result.pose.orientation = forward ? tf::createQuaternionMsgFromYaw(ttk::heading(start, end))
                                    : tf::createQuaternionMsgFromYaw(ttk::heading(end, start));
  result.header = start.header;
  return true;
}

double PathSmoother::enclosedArea(const geometry_msgs::PoseStamped& point_a,
                                  const geometry_msgs::PoseStamped& point_b,
                                  const geometry_msgs::PoseStamped& point_c)
{
  // Area of triangle https://www.mathopenref.com/coordtrianglearea.html
  return std::abs((point_a.pose.position.x * (point_b.pose.position.y - point_c.pose.position.y) +
                   point_b.pose.position.x * (point_c.pose.position.y - point_a.pose.position.y) +
                   point_c.pose.position.x * (point_a.pose.position.y - point_b.pose.position.y)) / 2);
}

double PathSmoother::curvature(const geometry_msgs::PoseStamped& point_a,
                               const geometry_msgs::PoseStamped& point_b,
                               const geometry_msgs::PoseStamped& point_c)
{
  // Curvature using triple points https://en.wikipedia.org/wiki/Menger_curvature
  double area = enclosedArea(point_a, point_b, point_c);
  double dist_ab = ttk::distance2D(point_a, point_b);
  double dist_bc = ttk::distance2D(point_b, point_c);
  double dist_ca = ttk::distance2D(point_c, point_a);
  return 4 * area / (dist_ab * dist_bc * dist_ca);
}

bool PathSmoother::insertInterpolatedIfValid(const geometry_msgs::PoseStamped& start,
                                             const geometry_msgs::PoseStamped& end, double weight,
                                             std::vector<geometry_msgs::PoseStamped>& pose_list)
{
  geometry_msgs::PoseStamped temp_pose;
  interpolate(start, end, weight, temp_pose);
  if (isStateValid(temp_pose))
  {
    pose_list.push_back(temp_pose);
    return true;
  }
  else
    return false;
}

void PathSmoother::publishPoses(const std::vector<geometry_msgs::PoseStamped>& poses, std::string ns,
                                double red, double green, double blue)
{
  visualization_msgs::MarkerArray path_marker;
  visualization_msgs::Marker clear_markers;
  clear_markers.action = visualization_msgs::Marker::DELETEALL;
  clear_markers.ns = ns;
  clear_markers.id = 0;
  path_marker.markers.push_back(clear_markers);
  int i = 1;
  for (auto& point : poses)
  {
    visualization_msgs::Marker pose_marker;
    pose_marker.header.frame_id = costmap_2d_->getGlobalFrameID();
    pose_marker.header.stamp = ros::Time::now();
    pose_marker.ns = ns;
    pose_marker.id = i;
    pose_marker.type = visualization_msgs::Marker::SPHERE;
    pose_marker.action = visualization_msgs::Marker::ADD;
    pose_marker.pose.position.x = point.pose.position.x;
    pose_marker.pose.position.y = point.pose.position.y;
    pose_marker.pose.orientation.w = 1;
    pose_marker.scale.x = 0.1;
    pose_marker.scale.y = 0.1;
    pose_marker.scale.z = 0.1;
    pose_marker.color.r = red;
    pose_marker.color.g = green;
    pose_marker.color.b = blue;
    pose_marker.color.a = 0.5;
    i++;
    path_marker.markers.push_back(pose_marker);
  }
  viz_result_pub_.publish(path_marker);
}

void PathSmoother::highlightPose(const geometry_msgs::PoseStamped& pose) const
{
  visualization_msgs::MarkerArray path_marker;
  visualization_msgs::Marker clear_markers;
  clear_markers.action = visualization_msgs::Marker::DELETEALL;
  clear_markers.ns = "highlight";
  clear_markers.id = 0;
  path_marker.markers.push_back(std::move(clear_markers));
  visualization_msgs::Marker pose_marker;
  pose_marker.header.frame_id = costmap_2d_->getGlobalFrameID();
  pose_marker.header.stamp = ros::Time::now();
  pose_marker.ns = "highlight";
  pose_marker.id = 0;
  pose_marker.type = visualization_msgs::Marker::SPHERE;
  pose_marker.action = visualization_msgs::Marker::ADD;
  pose_marker.pose = pose.pose;
  pose_marker.scale.x = 0.21;
  pose_marker.scale.y = 0.21;
  pose_marker.scale.z = 0.1;
  pose_marker.color.r = 1.0;
  pose_marker.color.a = 1.0;
  path_marker.markers.push_back(std::move(pose_marker));

  viz_result_pub_.publish(path_marker);
}

bool PathSmoother::connectWaypointsSrv(thorp_msgs::ConnectWaypoints::Request& req,
                                       thorp_msgs::ConnectWaypoints::Response& res)
{
  if (req.waypoints.empty())
  {
    ROS_ERROR("Path smoother: empty waypoints list requested");
    return true;
  }

  res.path.header = req.waypoints[0].header;
  res.path.poses = req.waypoints;

  // Path smoothing only makes sense for more than 2 waypoints
  if (req.waypoints.size() > 2 && !smoothBSpline(res.path, req.max_steps, req.max_radius))
  {
    ROS_ERROR("Path smoothing failed");
    return false;
  }

  // Path discretization only makes sense for more than 1 pose
  if (req.waypoints.size() > 1 && !rediscretize(res.path, req.resolution))
  {
    ROS_ERROR("Path discretization failed");
    return false;
  }

  if (req.visualize_path)
    publishPoses(res.path.poses, "discretized_path", 0.5, 0., 0.5);

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoother");

  ros::NodeHandle private_nh("~");
  PathSmoother path_smoother;
  ros::spin();

  return EXIT_SUCCESS;
}
