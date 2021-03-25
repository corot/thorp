#include <tf/tf.h>

#include <mbf_msgs/CheckPose.h>
#include <visualization_msgs/MarkerArray.h>

#include <thorp_msgs/ConnectWaypoints.h>
#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_navigation/waypoints_path.hpp"

WaypointsPath::WaypointsPath() : pnh_("~")
{
  allowed_space_ = pnh_.param("allowed_space", 2);  // FREE: 0, INSCRIBED: 1, LETHAL: 2, UNKNOWN: 3, OUTSIDE: 4
  path_resolution_ = pnh_.param("path_resolution", 0.1);
  if (path_resolution_ < 0.001)
  {
    ROS_ERROR("[waypoints path] Path resolution must be positive (%f provided)", path_resolution_);
    return;
  }
  viz_result_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("smoothed_path", 1, true);
  connect_wp_srv_ = pnh_.advertiseService("connect_waypoints", &WaypointsPath::connectWaypointsSrv, this);
  if (ros::service::waitForService("move_base_flex/check_pose_cost", ros::Duration(30)))
    check_pose_srv_ = nh_.serviceClient<mbf_msgs::CheckPose>("move_base_flex/check_pose_cost", true);
  else
    ROS_WARN("[waypoints path] MBF check pose service not available after 30s; collisions check disabled");

  ROS_INFO("[waypoints path] Waypoints path server ready");
}

bool WaypointsPath::subdivide(nav_msgs::Path& path)
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

bool WaypointsPath::rediscretize(nav_msgs::Path& path)
{
  if (path.poses.size() < 2)
  {
    ROS_ERROR("[waypoints path] At least 2 points in path is required");
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> new_states, temp_states;
  new_states.reserve(floor(3 * path.poses.size() / 2));
  new_states.push_back(path.poses[0]);
  for (unsigned int i = 1; i < path.poses.size(); ++i)
  {
    double length = ttk::distance2D(new_states.back(), path.poses.at(i));
    double step_size = path_resolution_ / length;
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

bool WaypointsPath::insertAtDistance(nav_msgs::Path& path, double distance)
{
  if (path.poses.size() < 2)
  {
    ROS_ERROR("[waypoints path] At least 2 points in path is required");
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

bool WaypointsPath::smoothBSpline(nav_msgs::Path& path, double max_steps, double max_radius)
{
  for (unsigned int idx = 1; idx < path.poses.size(); ++idx)
  {
    if (!isMotionValid(path.poses.at(idx - 1), path.poses.at(idx), true))
    {
      ROS_ERROR("[waypoints path] Input path is in collision, can't perform smoothing");
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
      if (isMotionValid(path.poses.at(i - 1), path.poses.at(i)) &&
          isMotionValid(path.poses.at(i), path.poses.at(i + 1)))  // Check motion bw path.poses.at(i - 1)
                                                                  // and path.poses[i - 1] for validity
      {
        interpolate(path.poses.at(i - 1), path.poses.at(i), 0.5, temp_pose_1);
        interpolate(path.poses.at(i), path.poses.at(i + 1), 0.5, temp_pose_2);
        interpolate(temp_pose_1, temp_pose_2, 0.5, temp_pose_1);
        if (isMotionValid(path.poses.at(i - 1), temp_pose_1) &&
            isMotionValid(temp_pose_1, path.poses.at(i + 1)))  // Check motion bw path.poses[i-1] and
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

bool WaypointsPath::isStateValid(const geometry_msgs::PoseStamped& pose, bool highlight_invalid_pose)
{
  if (!check_pose_srv_ || allowed_space_ >= mbf_msgs::CheckPoseResponse::OUTSIDE)
    return true;  // no check service available or any space is allowed, so why bothering to check?

  // Evaluate cost of the footprint at the given pose
  mbf_msgs::CheckPose srv;
  srv.request.pose = pose;
  srv.request.costmap = mbf_msgs::CheckPoseRequest::GLOBAL_COSTMAP;
  if (!check_pose_srv_.call(srv))
  {
    ROS_WARN("[waypoints path] MBF check pose service failed; assume pose is not valid");
    return false;
  }
  if (srv.response.state > allowed_space_)
  {
    if (highlight_invalid_pose)
    {
      // Highlight the problematic pose to allow easy debugging
      ROS_WARN("[waypoints path] Invalid state at %s: %c; cost: %ud",
               ttk::pose2cstr2D(pose), srv.response.state, srv.response.cost);
      highlightPose(pose);
    }
    return false;
  }
  return true;
}

bool WaypointsPath::isMotionValid(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
                                  bool highlight_invalid_pose)
{
  double length = ttk::distance2D(start, end);
  double step_size = path_resolution_ / length;
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

bool WaypointsPath::interpolate(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end,
                                double weight, geometry_msgs::PoseStamped& result)
{
  if (weight > 1 || weight < 0)
  {
    ROS_ERROR("[waypoints path] Weights should be within [0 1] range");
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

bool WaypointsPath::insertInterpolatedIfValid(const geometry_msgs::PoseStamped& start,
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

void WaypointsPath::publishPoses(const std::vector<geometry_msgs::PoseStamped>& poses,
                                 double red, double green, double blue)
{
  visualization_msgs::MarkerArray path_marker;
  visualization_msgs::Marker clear_markers;
  clear_markers.action = visualization_msgs::Marker::DELETEALL;
  clear_markers.ns = "discretized_path";
  clear_markers.id = 0;
  path_marker.markers.push_back(clear_markers);
  int i = 1;
  for (auto& point : poses)
  {
    visualization_msgs::Marker pose_marker;
    pose_marker.header.frame_id = "map";
    pose_marker.header.stamp = ros::Time::now();
    pose_marker.ns = "discretized_path";
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

void WaypointsPath::highlightPose(const geometry_msgs::PoseStamped& pose) const
{
  visualization_msgs::MarkerArray path_marker;
  visualization_msgs::Marker clear_markers;
  clear_markers.action = visualization_msgs::Marker::DELETEALL;
  clear_markers.ns = "highlight";
  clear_markers.id = 0;
  path_marker.markers.push_back(std::move(clear_markers));
  visualization_msgs::Marker pose_marker;
  pose_marker.header.frame_id = "map";
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

bool WaypointsPath::connectWaypointsSrv(thorp_msgs::ConnectWaypoints::Request& req,
                                       thorp_msgs::ConnectWaypoints::Response& res)
{
  ros::Time t0 = ros::Time::now();
  if (!check_pose_srv_)
  {
    // We use a persistent service client, so we must recheck that it's still there (but not block this call waiting!)
    if (ros::service::waitForService("move_base_flex/check_pose_cost", ros::Duration(0.1)))
      check_pose_srv_ = nh_.serviceClient<mbf_msgs::CheckPose>("move_base_flex/check_pose_cost", true);
    else
      ROS_WARN("[waypoints path] MBF check pose service not available; collisions check disabled");
  }

  if (req.waypoints.empty())
  {
    ROS_ERROR("[waypoints path] Empty waypoints list requested");
    return true;
  }

  res.path.header = req.waypoints[0].header;
  res.path.poses = req.waypoints;

  // Path smoothing only makes sense for more than 2 waypoints
  if (req.waypoints.size() > 2 && !smoothBSpline(res.path, req.max_steps, req.max_radius))
  {
    ROS_ERROR("[waypoints path] Path smoothing failed");
    return false;
  }

  // Path discretization only makes sense for more than 1 pose
  if (req.waypoints.size() > 1 && !rediscretize(res.path))
  {
    ROS_ERROR("[waypoints path] Path discretization failed");
    return false;
  }

  if (req.visualize_path)
    publishPoses(res.path.poses, 0.2, 0.4, 0.6);

  ROS_INFO("[waypoints path] Path created in %f seconds", (ros::Time::now() - t0).toSec());
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints_path");

  WaypointsPath waypoints_path;
  ros::spin();

  return EXIT_SUCCESS;
}
