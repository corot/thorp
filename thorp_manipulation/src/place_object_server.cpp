/*
 * Author: Jorge Santos
 */

#include <tf/tf.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit_msgs/PlaceLocation.h>

// auxiliary libraries
#include <thorp_toolkit/planning_scene.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

// other Thorp stuff
#include <thorp_msgs/ThorpError.h>

#include "thorp_manipulation/place_object_server.hpp"


namespace thorp_manipulation
{

PlaceObjectServer::PlaceObjectServer(const std::string& name) :
  as_(name, boost::bind(&PlaceObjectServer::executeCB, this, _1), false)
{
  ROS_INFO("[place object] Starting place action server...");

  // Register feedback callback for our server; executeCB is run on a separated thread, so it can be cancelled
  as_.registerPreemptCallback(boost::bind(&PlaceObjectServer::preemptCB, this));
  as_.start();
}

PlaceObjectServer::~PlaceObjectServer()
{
  as_.shutdown();
}

void PlaceObjectServer::executeCB(const thorp_msgs::PlaceObjectGoal::ConstPtr& goal)
{
  preempted_ = false;
  thorp_msgs::PlaceObjectResult result;

  if (!goal->object_name.empty() && goal->object_name != attached_object)
  {
    ROS_ERROR("[place object] Requested object not in gripper: '%s'; currently attached object is '%s'",
              goal->object_name.c_str(), attached_object.c_str());
    result.error.code = thorp_msgs::ThorpError::OBJECT_NOT_ATTACHED;
    result.error.text = mec2str(result.error.code);
    as_.setAborted(result);
    return;
  }

  ROS_INFO("[place object] Execute goal: place object '%s' on support surface '%s' at pose [%s]...",
           goal->object_name.c_str(), goal->support_surf.c_str(), ttk::pose2cstr3D(goal->place_pose));

  result.error.code = place(goal->object_name, goal->support_surf, goal->place_pose);
  result.error.text = mec2str(result.error.code);
  if (result.error.code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    as_.setSucceeded(result);
    attached_object = "";
  }
  else if (result.error.code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    as_.setPreempted(result);
  }
  else
  {
    as_.setAborted(result);
  }
}

void PlaceObjectServer::preemptCB()
{
  ROS_WARN("[place object] Action preempted; cancel all movement");
  preempted_ = true;
  gripper().stop();
  arm().stop();
}

int32_t PlaceObjectServer::place(const std::string& obj_name, const std::string& surface,
                                 const geometry_msgs::PoseStamped& pose)
{
  // Look for obj_name in the planning scene's list of attached collision objects
  geometry_msgs::PoseStamped attached_pose; geometry_msgs::Vector3 obj_size;
  int32_t result = ttk::getAttachedObjectData(obj_name, attached_pose, obj_size);
  if (result < 0)
  {
    // Error occurred while getting object data...
    return result;
  }

  ROS_INFO("[place object] Placing object '%s' at pose [%s]...", obj_name.c_str(), ttk::pose2cstr3D(pose));

  std::vector<moveit_msgs::PlaceLocation> locations;
  result = makePlaceLocations(pose, attached_pose, obj_size, obj_name, surface, locations);
  if (result < 0)
  {
    // Error occurred while making grasps...
    return result;
  }

  moveit_msgs::PlaceGoal goal = arm().constructPlaceGoal(obj_name, locations, false);
  goal.support_surface_name = surface;
  goal.allow_gripper_support_collision = true;

  // Allow some leeway in position (meters) and orientation (radians)
  arm().setGoalPositionTolerance(0.001);  // TODO: same values already set on parent class; add to the goal if needed
  arm().setGoalOrientationTolerance(0.02);

  // Allow replanning to increase the odds of a solution
  arm().allowReplanning(true);

  moveit::planning_interface::MoveItErrorCode place_result = arm().place(goal);

  if (preempted_)
  {
    ROS_WARN("[place object] Place action preempted");
    return moveit::planning_interface::MoveItErrorCode::PREEMPTED;
  }

  if (place_result)
  {
    ROS_INFO("[place object] Place succeeded!");
  }
  else
  {
    ROS_ERROR("[place object] Place fail with error code %d: %s", place_result.val, mec2str(place_result));
  }
  return place_result.val;
}

int32_t PlaceObjectServer::makePlaceLocations(const geometry_msgs::PoseStamped& target_pose,
                                              const geometry_msgs::PoseStamped& obj_pose,
                                              const geometry_msgs::Vector3& obj_size,
                                              const std::string& obj_name, const std::string& surface,
                                              std::vector<moveit_msgs::PlaceLocation>& place_locations)
{
  // Try up to PLACE_ATTEMPTS place locations with slightly different poses
  // target pose is expected to be the center of the object once placed in the support surface
  for (int attempt = 0; attempt < PLACE_ATTEMPTS; ++attempt)
  {
    moveit_msgs::PlaceLocation l;
    l.place_pose = target_pose;
    l.place_pose.pose.position.z += obj_size.z / 2.0;  // gripper pose is at the top of the object, so we must add z/2
    if (!validateTargetPose(l.place_pose, true, true, true, attempt))
      return thorp_msgs::ThorpError::INVALID_TARGET_POSE;

    // MoveGroup::place will transform the provided place pose with the attached body pose, so the object retains
    // the orientation it had when picked. However, with our 4-dofs arm this is infeasible (nor we care about the
    // objects orientation!), so we cancel this transformation. It is applied here:
    // https://github.com/ros-planning/moveit_ros/blob/jade-devel/manipulation/pick_place/src/place.cpp#L64
    // More details on this issue: https://github.com/ros-planning/moveit_ros/issues/577
    tf::Transform place_tf, aco_tf;
    tf::poseMsgToTF(l.place_pose.pose, place_tf);
    tf::poseMsgToTF(obj_pose.pose, aco_tf);
    tf::poseTFToMsg(place_tf * aco_tf, l.place_pose.pose);
    ROS_DEBUG("[place object] Compensate place pose with the attached object pose [%s]. Results: [%s]",
              ttk::pose2cstr3D(obj_pose.pose), ttk::pose2cstr3D(l.place_pose.pose));

    l.pre_place_approach.direction.vector.x = +1;
    l.pre_place_approach.direction.header.frame_id = arm().getEndEffectorLink();
    l.pre_place_approach.min_distance = 0.01;  // TODO probably dangerous; make proportional to the pitch
    l.pre_place_approach.desired_distance = 0.1;

    l.post_place_retreat.direction.vector.x = -1;
    l.post_place_retreat.direction.header.frame_id = arm().getEndEffectorLink();
    l.post_place_retreat.min_distance = 0.01;
    l.post_place_retreat.desired_distance = 0.1;

    l.post_place_posture.joint_names.push_back("gripper_joint");
    l.post_place_posture.points.resize(1);
    l.post_place_posture.points[0].positions.push_back(gripper_open);

    l.allowed_touch_objects.push_back(obj_name);
    l.allowed_touch_objects.push_back(surface);

    l.id = std::to_string(attempt);

    place_locations.push_back(l);

    ROS_DEBUG("[place object] Place attempt %d at pose [%s]...", attempt, ttk::pose2cstr3D(l.place_pose));
  }

  return place_locations.size();
}

};
