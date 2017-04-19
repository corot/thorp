/*
 * Author: Jorge Santos
 */

#include <tf/tf.h>
#include <ros/ros.h>

// auxiliary libraries
#include <yocs_math_toolkit/common.hpp>
#include <geometric_shapes/shape_operations.h>

// MoveIt!
#include <moveit_msgs/PlaceLocation.h>

// Thorp stuff
#include "thorp_arm_ctrl/place_object_server.hpp"


namespace thorp_arm_ctrl
{

PlaceObjectServer::PlaceObjectServer(const std::string name) :
  as_(name, false), action_name_(name)
{
  // Register the goal and feedback callbacks
  as_.registerGoalCallback(boost::bind(&PlaceObjectServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlaceObjectServer::preemptCB, this));
  as_.start();
}

PlaceObjectServer::~PlaceObjectServer()
{
  as_.shutdown();
}

void PlaceObjectServer::goalCB()
{
  ROS_INFO("[place object] Received goal!");

  thorp_msgs::PlaceObjectGoalConstPtr goal = as_.acceptNewGoal();

  arm().setSupportSurfaceName(goal->support_surf);

  // Allow some leeway in position (meters) and orientation (radians)
  arm().setGoalPositionTolerance(0.001);
  arm().setGoalOrientationTolerance(0.02);

  // Allow replanning to increase the odds of a solution
  arm().allowReplanning(true);

  thorp_msgs::PlaceObjectResult result;
  result.error.code = place(goal->object_name, goal->support_surf, goal->place_pose);
  result.error.text = mec2str(result.error.code);
  if (result.error.code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    as_.setSucceeded(result);
  }
  else
  {
    // Ensure we don't retain any object attached to the gripper
    arm().detachObject(goal->object_name);
    setGripper(gripper_open, false);

    as_.setAborted(result);
  }
}

void PlaceObjectServer::preemptCB()
{
  ROS_WARN("[place object] %s: Preempted", action_name_.c_str());
  gripper().stop();
  arm().stop();

  // set the action state to preempted
  as_.setPreempted();
}

int32_t PlaceObjectServer::place(const std::string& obj_name, const std::string& surface,
                              const geometry_msgs::PoseStamped& pose)
{
  // Look for obj_name in the list of attached objects
  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
      planningScene().getAttachedObjects(std::vector<std::string>(1, obj_name));

  if (objects.size() == 0)
  {
    // Maybe pick failed; we will not continue because place will surely fail without knowing the attaching pose
    ROS_ERROR("[place object] Attached collision object '%s' not found", obj_name.c_str());
    return thorp_msgs::ThorpError::OBJECT_NOT_FOUND;
  }

  if (objects.size() > 1)
  {
    // This should not happen... we grasped two objects with the same name???
    ROS_WARN("[place object] More than one (%d) attached collision objects with name '%s' found!",
             objects.size(), obj_name.c_str());
  }

  // We just need object's pose so we can subtract its pose from the place location poses
  geometry_msgs::Pose aco_pose;  // No stamped; it's relative to attaching frame (gripper_link)
  const moveit_msgs::AttachedCollisionObject& aco = objects[obj_name];

  if (aco.object.primitive_poses.size() > 0)
  {
    aco_pose = aco.object.primitive_poses[0];
  }
  else if (aco.object.mesh_poses.size() > 0)
  {
    aco_pose = aco.object.mesh_poses[0];
  }
  else
  {
    ROS_ERROR("[place object] Attached collision object '%s' has no pose!", obj_name.c_str());
    return thorp_msgs::ThorpError::OBJECT_POSE_NOT_FOUND;
  }

  ROS_INFO("[place object] Placing object '%s' at pose [%s]...", obj_name.c_str(), mtk::pose2str3D(pose).c_str());

  // Try up to PLACE_ATTEMPTS place locations with slightly different poses
  moveit::planning_interface::MoveItErrorCode result;

  for (int attempt = 0; attempt < PLACE_ATTEMPTS; ++attempt)
  {
    geometry_msgs::PoseStamped p = pose;
    if (!validateTargetPose(p, true, attempt))
    {
      return thorp_msgs::ThorpError::INVALID_TARGET_POSE;
    }

    // MoveGroup::place will transform the provided place pose with the attached body pose, so the object retains
    // the orientation it had when picked. However, with our 4-dofs arm this is infeasible (nor we care about the
    // objects orientation!), so we cancel this transformation. It is applied here:
    // https://github.com/ros-planning/moveit_ros/blob/jade-devel/manipulation/pick_place/src/place.cpp#L64
    // More details on this issue: https://github.com/ros-planning/moveit_ros/issues/577
    tf::Transform place_tf, aco_tf;
    tf::poseMsgToTF(p.pose, place_tf);
    tf::poseMsgToTF(aco_pose, aco_tf);
    tf::poseTFToMsg(place_tf * aco_tf, p.pose);

    ROS_DEBUG("[place object] Compensate place pose with the attached object pose [%s]. Results: [%s]",
              mtk::pose2str3D(aco_pose).c_str(), mtk::pose2str3D(p.pose).c_str());

    ROS_DEBUG("[place object] Place attempt %d at pose [%s]...", attempt, mtk::pose2str3D(p).c_str());

    moveit_msgs::PlaceLocation l;
    l.place_pose = p;

    l.pre_place_approach.direction.vector.x = 0.5;
    l.pre_place_approach.direction.header.frame_id = arm().getEndEffectorLink();
    l.pre_place_approach.min_distance = 0.005;
    l.pre_place_approach.desired_distance = 0.1;

    l.post_place_retreat.direction.vector.x = -0.5;
    l.post_place_retreat.direction.header.frame_id = arm().getEndEffectorLink();
    l.post_place_retreat.min_distance = 0.005;
    l.post_place_retreat.desired_distance = 0.1;

    l.post_place_posture.joint_names.push_back("gripper_joint");
    l.post_place_posture.points.resize(1);
    l.post_place_posture.points[0].positions.push_back(gripper_open);

    l.allowed_touch_objects.push_back(obj_name);
    l.allowed_touch_objects.push_back(surface);

    l.id = attempt;

    std::vector<moveit_msgs::PlaceLocation> locs(1, l);

    if ((result = arm().place(obj_name, locs)))
    {
      ROS_INFO("[place object] Place successfully completed");
      return result.val;
    }

    ROS_DEBUG("[place object] Place attempt %d failed: %s", attempt, mec2str(result));
  }

  ROS_ERROR("[place object] Place failed after %d attempts", PLACE_ATTEMPTS);
  return result.val;
}

};
