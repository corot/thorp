#!/usr/bin/env python

# system
import sys
import math
# ros basics
import roslib; roslib.load_manifest('thorp_smach')
import rospy
import smach
import tf
from tf import TransformListener
#from tf.transformations import quaternion_from_euler

import control_msgs
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryResult
import geometry_msgs.msg as geometry_msgs
import trajectory_msgs
from trajectory_msgs.msg import JointTrajectoryPoint


'''
General
'''
@smach.cb_interface(input_keys=['trajectory_goal'],
                    outcomes=['succeeded',
                              'aborted'])
def generalGoalCb(userdata, goal):
    goal = userdata.trajectory_goal
    rospy.loginfo('Preparing FollowJointTrajectoryGoal ...')
    rospy.loginfo('FollowJointTrajectoryGoal prepared.')
    return goal

@smach.cb_interface(output_keys=['error_code'],
                    outcomes=['succeeded',
                              'aborted'])
def generalResponseCb(userdata, status, result):
    userdata.error_code = result.error_code
    if result.error_code == control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL:
        rospy.loginfo('Trajectory action succeeded.')
        return 'succeeded'
    else:
        rospy.loginfo('Trajectory action failed. Error code is ' + str(result.error_code))
        return 'aborted'

'''
Gripper control
'''
@smach.cb_interface(input_keys=['angle'])
def gripperControlGoalCb(userdata, goal):
    rospy.loginfo('Preparing gripper goal ...')
    gripper_goal = control_msgs.msg.FollowJointTrajectoryGoal()
    trajectory = trajectory_msgs.msg.JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = "base_footprint"
    trajectory.joint_names.append("gripper")
    waypoint = trajectory_msgs.msg.JointTrajectoryPoint()
    waypoint.positions.append(userdata.angle)
    waypoint.velocities.append(0.0)
    waypoint.accelerations.append(0.0)
    trajectory.points.append(waypoint)
    gripper_goal.trajectory = trajectory
    rospy.loginfo('Gripper goal prepared.')
    return gripper_goal

@smach.cb_interface(input_keys=['open_gripper',
                                'close_gripper'])
def gripperControlResultCb(userdata, status, result):
    if result.error_code == control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL:
        if userdata.open_gripper:
            rospy.loginfo("Gripper opened.")
            return 'succeeded'
        elif userdata.close_gripper:
            rospy.loginfo("Gripper closed.")
            return 'succeeded'
        else:
            return 'succeeded'
    else:
        print 'error_code is ' + str(result.error_code)
        return 'aborted'

'''
Head control callbacks
'''
@smach.cb_interface(input_keys=['tf_listener',
                                'goal_pose'],
                    output_keys=['tf_listener'])
def headControlRequestCb(userdata, goal):
    rospy.loginfo('Preparing head goal ...')
    head_target = geometry_msgs.PoseStamped()
    pose_new_stamp = userdata.goal_pose
    try:
        pose_new_stamp.header.stamp = userdata.tf_listener.getLatestCommonTime("sensor_3d_fixed_link",
                                                                               userdata.goal_pose.header.frame_id)
        head_target = userdata.tf_listener.transformPose("sensor_3d_fixed_link", pose_new_stamp)
    except tf.Exception, e:
        rospy.logerr('Couldn`t transform requested pose!')
        rospy.logerr('%s', e)
    pan_angle = math.atan2(head_target.pose.position.y, head_target.pose.position.x)
    tilt_angle = math.atan2(head_target.pose.position.z, head_target.pose.position.x)
    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory.header.frame_id = "move_head"
    head_goal.trajectory.header.stamp = rospy.Time.now()
    head_goal.trajectory.joint_names.append("head_pan")
    head_goal.trajectory.joint_names.append("head_tilt")
    waypoint = JointTrajectoryPoint()
    waypoint.positions.append(pan_angle)
    waypoint.positions.append(tilt_angle)
    waypoint.velocities.append(0.5)
    waypoint.velocities.append(0.5)
    waypoint.accelerations.append(0.0)
    waypoint.accelerations.append(0.0)
    head_goal.trajectory.points.append(waypoint)
    rospy.loginfo('Head goal prepared.')
    return head_goal

'''
    Move arm up
'''
@smach.cb_interface(input_keys=['joint_states',
                                'distance'])
def moveArmUpGoalCb(userdata, goal):
    arm_goal = control_msgs.msg.FollowJointTrajectoryGoal()
    trajectory = trajectory_msgs.msg.JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = ""
    trajectory.joint_names.append("torso_turn")
    trajectory.joint_names.append("torso_lift")
    trajectory.joint_names.append("shoulder")
    trajectory.joint_names.append("elbow")
    trajectory.joint_names.append("wrist")
    waypoint = trajectory_msgs.msg.JointTrajectoryPoint()
    waypoint.positions.append(userdata.joint_states.position[2])
    waypoint.positions.append(userdata.joint_states.position[3] + userdata.distance)
    waypoint.positions.append(userdata.joint_states.position[4])
    waypoint.positions.append(userdata.joint_states.position[5])
    waypoint.positions.append(userdata.joint_states.position[6])
    waypoint.velocities.append(0.0)
    waypoint.accelerations.append(0.0)
    trajectory.points.append(waypoint)
    arm_goal.trajectory = trajectory
    rospy.loginfo('Move arm up goal prepared.')
    return arm_goal


class FollowJointTrajectoryErrorCodesParser(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'parsed'],
                             input_keys=['error_code'],
                             output_keys=['error_message'])
        self.error_code_dict = {control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL:'SUCCESSFUL',
                                control_msgs.msg.FollowJointTrajectoryResult.INVALID_GOAL:'INVALID_GOAL',
                                control_msgs.msg.FollowJointTrajectoryResult.INVALID_JOINTS:'INVALID_JOINTS',
                                control_msgs.msg.FollowJointTrajectoryResult.OLD_HEADER_TIMESTAMP:'OLD_HEADER_TIMESTAMP',
                                control_msgs.msg.FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:'PATH_TOLERANCE_VIOLATED',
                                control_msgs.msg.FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:'GOAL_TOLERANCE_VIOLATED'}
    def execute(self, userdata):
        error_message = str()
        if userdata.error_code in self.error_code_dict:
            error_message = self.error_code_dict[userdata.error_code]
            userdata.error_message = error_message
            rospy.loginfo("FollowJointTrajectory finished with error code: "
                          + self.error_code_dict[userdata.error_code])
            if userdata.error_code is control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL:
                return 'success'
        else:
            error_message = str("Error code '" + str(userdata.error_code) + "' not in dictionary. "
                                 + "Check FollowJointTrajectory action description for more information!")
            userdata.error_message = error_message
            rospy.loginfo("Error code '" + str(userdata.error_code) + "' not in dictionary. "
                          + "Check FollowJointTrajectory action description for more information!")
        return 'parsed'
