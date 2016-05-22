/*
 * Author: Jorge Santos
 */

#include "thorp_arm_ctrl/move_to_target_server.hpp"


namespace thorp_arm_ctrl
{


MoveToTargetServer::MoveToTargetServer(const std::string name) :
    action_name_(name), as_(name, boost::bind(&MoveToTargetServer::executeCB, this, _1), false)
{
  // Register feedback callback for our server; executeCB is run on a separated thread, so it can be cancelled
  as_.registerPreemptCallback(boost::bind(&MoveToTargetServer::preemptCB, this));
  as_.start();
}

MoveToTargetServer::~MoveToTargetServer()
{
  as_.shutdown();
}

void MoveToTargetServer::executeCB(const thorp_msgs::MoveToTargetGoal::ConstPtr& goal)
{
  ROS_INFO("[move to target] Received goal!");

  thorp_msgs::MoveToTargetResult result;

  switch (goal->target_type)
  {
    case thorp_msgs::MoveToTargetGoal::NAMED_TARGET:
      result.error.code = moveArmTo(goal->named_target);
      break;
    case thorp_msgs::MoveToTargetGoal::JOINT_STATE:
      result.error.code = moveArmTo(goal->joint_state);
      break;
    case thorp_msgs::MoveToTargetGoal::POSE_TARGET:
      result.error.code = moveArmTo(goal->pose_target);
      break;
    default:
      result.error.code = thorp_msgs::ThorpError::INVALID_TARGET_TYPE;
      ROS_ERROR("[move to target] Move to target of type %d not implemented", goal->target_type);
      break;
  }

  result.error.text = mec2str(result.error);
  if (result.error.code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR("SUCCCCCCCCCCCCCCCCCCC   %d", result.error.code);
    as_.setSucceeded(result);
  }
  else if (result.error.code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_ERROR("PREEEEEEEEEEEEEEEEMPT   %d", result.error.code);
    as_.setPreempted();
  }
  else
  {
    ROS_ERROR("ABOOOOOOOOOOOOOOOOORT   %d", result.error.code);
    as_.setAborted(result);
  }
}

void MoveToTargetServer::preemptCB()
{
  ROS_INFO("[move to target] Action preempted; cancel all movement");
  gripper().stop();
  arm().stop();
}

int32_t MoveToTargetServer::moveArmTo(const std::string& target)
{
  ROS_DEBUG("[move to target] Move arm to '%s' position", target.c_str());
  if (arm().setNamedTarget(target) == false)
  {
    ROS_ERROR("[move to target] Set named target '%s' failed", target.c_str());
    return thorp_msgs::ThorpError::INVALID_NAMED_TARGET;
  }

  moveit::planning_interface::MoveItErrorCode result = arm().move();
  if (result)
  {
    ROS_INFO("[move to target] Move to target '%s' completed", target.c_str());
  }
  else if (as_.isPreemptRequested())
  {
    ROS_WARN("[move to target] Move to target '%s' preempted", target.c_str());
    result.val = moveit::planning_interface::MoveItErrorCode::PREEMPTED;
  }
  else
  {
    ROS_ERROR("[move to target] Move to target '%s' failed: %s", target.c_str(), mec2str(result));
  }

  return result.val;
}

int32_t MoveToTargetServer::moveArmTo(const sensor_msgs::JointState& target)
{
  ROS_DEBUG_STREAM("[move to target] Move arm to target configuration:\n" << target);
  if (arm().setJointValueTarget(target) == false)
  {
    ROS_ERROR_STREAM("[move to target] Set joint value target failed:\n" << target);
    return thorp_msgs::ThorpError::INVALID_JOINT_STATE;
  }

  moveit::planning_interface::MoveItErrorCode result = arm().move();
  if (result)
  {
    ROS_INFO("[move to target] Move to joint value target completed");
  }
  else if (as_.isPreemptRequested())
  {
    ROS_WARN("[move to target] Move to joint value target preempted");
    result.val = moveit::planning_interface::MoveItErrorCode::PREEMPTED;
  }
  else
  {
    ROS_ERROR("[move to target] Move to joint value target failed: %s", mec2str(result));
  }

  return result.val;
}

int32_t MoveToTargetServer::moveArmTo(const geometry_msgs::PoseStamped& target)
{
  ROS_DEBUG("[move to target] Move arm to [%s]", mtk::pose2str3D(target.pose).c_str());

  geometry_msgs::PoseStamped modiff_target = target;
  if (!validateTargetPose(modiff_target, true, false, false))
  {
    return thorp_msgs::ThorpError::INVALID_TARGET_POSE;
  }

  if (arm().setPoseTarget(modiff_target) == false)
  {
    ROS_ERROR("[move to target] Set pose target [%s] failed", mtk::pose2str3D(modiff_target.pose).c_str());
    return thorp_msgs::ThorpError::INVALID_TARGET_POSE;
  }

  moveit::planning_interface::MoveItErrorCode result = arm().move();
  if (result)
  {
    ROS_INFO("[move to target] Move to target [%s] completed", mtk::pose2str3D(modiff_target.pose).c_str());
  }
  else if (as_.isPreemptRequested())
  {
    ROS_WARN("[move to target] Move to target [%s] preempted", mtk::pose2str3D(modiff_target.pose).c_str());
    result.val = moveit::planning_interface::MoveItErrorCode::PREEMPTED;
  }
  else
  {
    ROS_ERROR("[move to target] Move to target [%s] failed: %s", mtk::pose2str3D(modiff_target.pose).c_str(),
              mec2str(result));
  }

  return result.val;
}


bool MoveToTargetServer::setGripper(float opening, bool wait_for_complete)
{
  ROS_DEBUG("[move to target] Set gripper opening to %f", opening);
  if (gripper().setJointValueTarget("gripper_joint", opening) == false)
  {
    ROS_ERROR("[move to target] Set gripper opening to %f failed", opening);
    return false;
  }

  moveit::planning_interface::MoveItErrorCode result =
      wait_for_complete ? gripper().move() : gripper().asyncMove();
  if (result == true)
  {
    return true;
  }
  else
  {
    ROS_ERROR("[move to target] Set gripper opening failed (error %d)", result.val);
    return false;
  }
}


};
