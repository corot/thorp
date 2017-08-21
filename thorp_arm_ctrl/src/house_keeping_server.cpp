/*
 * Author: Jorge Santos
 */

#include <moveit_msgs/GetPlanningScene.h>

#include "thorp_arm_ctrl/house_keeping_server.hpp"


namespace thorp_arm_ctrl
{


HouseKeepingServer::HouseKeepingServer()
{
  ros::NodeHandle nh;

  arm_trajectory_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 1);
  force_resting_srv_  = nh.advertiseService("force_resting", &HouseKeepingServer::forceRestingCB, this);
  clear_gripper_srv_  = nh.advertiseService("clear_gripper", &HouseKeepingServer::clearGripperCB, this);

  // Get default planning scene so I can restore it after temporal changes
  planning_scene_srv_ = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  if (planning_scene_srv_.waitForExistence(ros::Duration(30.0)))
  {
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components |= moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    if (planning_scene_srv_.call(srv))
    {
      planning_scene_ = srv.response.scene;
    }
    else
    {
      ROS_ERROR("[house keeping] Failed to call service get_planning_scene");
    }
  }
  else
  {
    ROS_ERROR("[house keeping] Service get_planning_scene not available after 30s");
  }
}

HouseKeepingServer::~HouseKeepingServer()
{
}


bool HouseKeepingServer::forceRestingCB(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response)
{
  if (planning_scene_.allowed_collision_matrix.entry_values.empty())
  {
    ROS_ERROR("[house keeping] Not master planning_scene stored, so cannot alter it");
    return false;
  }

  ROS_INFO("[house keeping] Executing force resting service");

  moveit_msgs::PlanningScene ps = planning_scene_;
  ps.is_diff = true;
  for (auto& entry_values : ps.allowed_collision_matrix.entry_values)
    for (auto& value : entry_values.enabled)
      value = true;

  bool moved = false;
  if (planning_scene_interface_.applyPlanningScene(ps))
  {
    if (arm().setNamedTarget("resting"))
    {
      moveit::planning_interface::MoveItErrorCode result = arm().move();
      if (result)
      {
        ROS_INFO("[house keeping] Move to target 'resting' completed");
        moved = true;
      }
      else
      {
        ROS_ERROR("[house keeping] Move to target 'resting' failed: %s", mec2str(result));
      }
    }
    else
    {
      ROS_ERROR("[house keeping] Set named target 'resting' failed");
    }
  }
  else
  {
    ROS_ERROR("[house keeping] Apply relaxed planning scene failed");
  }

  if (!planning_scene_interface_.applyPlanningScene(planning_scene_))
  {
    ROS_ERROR("[house keeping] Restore original planning scene failed");
  }

  return moved;


// PREVIOUS ATTEMPTS
///  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ptr;
//  planning_scene_monitor::LockedPlanningSceneRW lps(planning_scene_monitor_ptr_);
//  collision_detection::AllowedCollisionMatrix acm = lps->getAllowedCollisionMatrix();
//  lps->getAllowedCollisionMatrixNonConst().setEntry(true);
//  lps->getAllowedCollisionMatrixNonConst().setEntry("arm_mount_link", "arm_elbow_flex_servo_link", true);
//
//  acm.print(std::cout);
//  ROS_ERROR_STREAM(""<< acm.getSize());
//  ROS_ERROR_STREAM(""<< acm.getSize());
//  ROS_ERROR_STREAM(""<< acm.getSize());
//  ROS_ERROR_STREAM(""<< acm.getSize());
//  lps->getAllowedCollisionMatrixNonConst().print(std::cout);

//  this kinda works
  std::map<std::string, double> joints = arm().getNamedTargetValues("resting");
  if (joints.size() < 4)
  {
    ROS_ERROR("[house keeping] 'resting' named target only contains %lu joints (at least 4 required)", joints.size());
    return false;
  }

  // send a goal to the action
  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp = ros::Time::now();
  traj.points.resize(1);
  traj.points[0].time_from_start = ros::Duration(5.0);
  for (auto joint : joints)
  {
    if (joint.first.find("arm_") == 0)
    {
      ROS_ERROR_STREAM(""<< joint.first<<"   "<< joint.second);
      traj.joint_names.push_back(joint.first);
      traj.points[0].positions.push_back(joint.second);
    }
  }

  arm_trajectory_pub_.publish(traj);
  return true;
}

bool HouseKeepingServer::clearGripperCB(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response)
{
  ROS_INFO("[house keeping] Ensure we don't retain any object attached to the gripper");

  arm().detachObject();
  return setGripper(gripper_open, false);
}

};
