#!/usr/bin/env python
import math
import tf
from thorp_smach.state_machines.state_machines_imports import *
from thorp_smach.pick_and_place_tools.msg_imports import *


@smach.cb_interface(input_keys=['goal_pose',
                                'goal_link_name',
                                'robot_state',
                                'avoid_collisions'])
def getPositionIKRequestCb(userdata, request):
    new_request = moveit_srvs.GetPositionIKRequest()
    new_request.ik_request.group_name = "arm"
    new_request.ik_request.ik_link_name = userdata.goal_link_name #optional
    goal_pose = geometry_msgs.PoseStamped()
    goal_pose.header.stamp = rospy.Time.now();
    goal_pose.header.frame_id = userdata.goal_pose.header.frame_id
    angle = math.atan2(userdata.goal_pose.pose.position.y, userdata.goal_pose.pose.position.x)
    dist = math.sqrt(math.pow(userdata.goal_pose.pose.position.x, 2) + math.pow(userdata.goal_pose.pose.position.y, 2))
    goal_pose.pose.position.x = dist * math.cos(angle)
    goal_pose.pose.position.y = dist * math.sin(angle)
    goal_pose.pose.position.z = userdata.goal_pose.pose.position.z
    yaw = angle
    roll = math.pi / 2
    pitch = 0.0
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    goal_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
    new_request.ik_request.pose_stamped = goal_pose
    new_request.ik_request.robot_state = userdata.robot_state;
    new_request.ik_request.timeout = rospy.Duration(0.5)
    new_request.ik_request.avoid_collisions = userdata.avoid_collisions;
    rospy.loginfo("'Get Position IK' service call prepared.")
    return new_request

@smach.cb_interface(outcomes=['succeeded',
                              'failed'],
                    input_keys=['error_code',
                                'error_message'],
                    output_keys=['error_code'])
def getPositionIKResponseGoalCb(userdata, response):
    userdata.error_code = response.error_code.val
    if response.error_code.val == moveit_msgs.MoveItErrorCodes.SUCCESS:
        rospy.loginfo("IK solution found.")
        return 'succeeded'
    else:
        rospy.loginfo("No IK solution found.")
        return 'failed'

@smach.cb_interface(outcomes=['succeeded',
                              'failed'],
                    input_keys=['error_code'],
                    output_keys=['arm_control_goal',
                                 'error_code'])
def getPositionIKResponseArmControllerGoalCb(userdata, response):
    userdata.error_code = response.error_code.val
    arm_control_goal = control_msgs.FollowJointTrajectoryGoal()
    if response.error_code.val == moveit_msgs.MoveItErrorCodes.SUCCESS:
        rospy.loginfo("IK solution found.") 
        trajectory = trajectory_msgs.JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = "base_footprint"
        for name in response.solution.joint_state.name:
            trajectory.joint_names.append(name)
        waypoint = trajectory_msgs.JointTrajectoryPoint()
        for position in response.solution.joint_state.position:
            waypoint.positions.append(position)
            waypoint.velocities.append(0.25)
            waypoint.accelerations.append(0.0)
        trajectory.points.append(waypoint)
        arm_control_goal.trajectory = trajectory
        userdata.arm_control_goal = arm_control_goal
        rospy.loginfo("IK solution found.") 
        return 'succeeded'
    else:
        rospy.loginfo("No IK solution found.")
        return 'failed'
    
@smach.cb_interface(output_keys=['robot_goal_state',
                                 'error_code'],
                    outcomes=['succeeded',
                              'failed'])
def getPositionIKResponseRobotStateCb(userdata, response):
    userdata.error_code = response.error_code.val
    if response.error_code.val is moveit_msgs.MoveItErrorCodes.SUCCESS:
        rospy.loginfo("IK solution found.") 
        userdata.robot_goal_state = response.solution
        return 'succeeded'
    else:
        return 'failed'

class PrepareIKSeed(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success',
                                       'error'],
                             input_keys=['zero',
                                         'default',
                                         'bottom',
                                         'robot_state'],
                             output_keys=['robot_state'])

    def execute(self, userdata):
        userdata.robot_state.joint_state.position = list(userdata.robot_state.joint_state.position);
        if userdata.zero:
            rospy.loginfo('IK seed is zero robot state.')
            for name in userdata.robot_state.joint_state.name:
                joint_nr = userdata.robot_state.joint_state.name.index(name)
                userdata.robot_state.joint_state.position[joint_nr] = 0.0
        elif userdata.default:
            rospy.loginfo('IK seed is default robot state.')
            for name in userdata.robot_state.joint_state.name:
                joint_nr = userdata.robot_state.joint_state.name.index(name)
                if name is 'shoulder':
                    userdata.robot_state.joint_state.position[joint_nr] = 1.57
                elif name is 'elbow':
                    userdata.robot_state.joint_state.position[joint_nr] = -3.14
                elif name is 'wrist':
                    userdata.robot_state.joint_state.position[joint_nr] = 1.57
                else:
                    userdata.robot_state.joint_state.position[joint_nr] = 0.0
        elif userdata.bottom:
            rospy.loginfo('IK seed is bottom robot state.')
            for name in userdata.robot_state.joint_state.name:
                joint_nr = userdata.robot_state.joint_state.name.index(name)
                if name is 'shoulder':
                    userdata.robot_state.joint_state.position[joint_nr] = 0.0
                elif name is 'elbow':
                    userdata.robot_state.joint_state.position[joint_nr] = -1.3
                elif name is 'wrist':
                    userdata.robot_state.joint_state.position[joint_nr] = 1.3
                else:
                    userdata.robot_state.joint_state.position[joint_nr] = 0.0
        else:
            rospy.loginfo('IK seed is current robot state.')
        return 'success'
