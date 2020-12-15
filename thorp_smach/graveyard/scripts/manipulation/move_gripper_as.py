#!/usr/bin/env python

###
# Simplified interface for opening and closing Korus' gripper
###

# system
import sys
import math
# ros basics & smach
import roslib; roslib.load_manifest('thorp_smach')
import rospy
import smach
from smach import StateMachine
import smach_ros
from smach_ros import SimpleActionState
from smach_ros import ServiceState
from smach_ros import ActionServerWrapper

from pick_and_place_msgs.msg import MoveGripperAction
from pick_and_place_msgs.msg import MoveGripperGoal
from pick_and_place_msgs.msg import MoveGripperFeedback
from pick_and_place_msgs.msg import MoveGripperResult

from thorp_smach.pick_and_place_tools import ik, move_arm, trajectory_control, misc_tools

import trajectory_msgs, kinematics_msgs, control_msgs, arm_navigation_msgs


class ParseGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=['incoming_goal'],
                             output_keys=['feedback',
                                          'gripper_goal'])

    def execute(self, userdata):
        rospy.loginfo('Preparing gripper goal ...')
        gripper_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        trajectory = trajectory_msgs.msg.JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = "base_footprint"
        trajectory.joint_names.append("gripper")
        waypoint = trajectory_msgs.msg.JointTrajectoryPoint()
        waypoint.positions.append(userdata.incoming_goal.gripper_angle)
        waypoint.velocities.append(0.5)
        waypoint.accelerations.append(0.0)
        trajectory.points.append(waypoint)
        gripper_goal.trajectory = trajectory
        userdata.gripper_goal = gripper_goal
        feedback = MoveGripperFeedback()
        feedback.process_state = 'Finished parsing MoveGripperGoal'
        userdata.feedback = feedback
        rospy.loginfo('Gripper goal prepared.')
        return 'success'
    
class Finalise(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success',
                                       'error'],
                             input_keys=['error_message',
                                         'error_code'],
                             output_keys=['feedback',
                                          'result'])

    def execute(self, userdata):
        rospy.loginfo('Preparing MoveGripperResult ...')
        result = MoveGripperResult()
        result.error_message = userdata.error_message
        result.error_code = userdata.error_code
        userdata.result = result
        feedback = MoveGripperFeedback()
        feedback.process_state = 'Finished move gripper action.'
        userdata.feedback = feedback
        rospy.loginfo('MoveGripperResult prepared.')
        if userdata.error_code is control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL:
            return 'success'
        else:
            return 'error'
    
    
def main():
    rospy.init_node('move_gripper_as')
    
    sm = smach.StateMachine(outcomes=['success',
                                      'preempted',
                                      'error'],
                            input_keys=['goal',
                                        'feedback',
                                        'result'],
                            output_keys=['feedback',
                                         'result'])
    
    sm.userdata.gripper_control_goal = control_msgs.msg.FollowJointTrajectoryGoal()
    sm.userdata.motors = ['gripper']
    
    with sm:
        smach.StateMachine.add('ParseGoal',
                               ParseGoal(),
                               remapping={'incoming_goal':'goal',
                                          'feedback':'feedback',
                                          'gripper_goal':'gripper_control_goal'},
                               transitions={'success':'EnableMotors'})
        
        smach.StateMachine.add('EnableMotors',
                               misc_tools.EnableMotors(),
                               transitions={'success':'MoveGripper'},
                               remapping={'motors':'motors'})
        
        smach.StateMachine.add('MoveGripper',
                               SimpleActionState('gripper_controller',
                                                 control_msgs.msg.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.generalGoalCb,
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 input_keys=['gripper_control_goal'],
                                                 output_keys=['error_code']),
                               remapping={'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveGripperErrorCode',
                                            'aborted':'ParseMoveGripperErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('ParseMoveGripperErrorCode',
                               trajectory_control.FollowJointTrajectoryErrorCodesParser(),
                               transitions={'success':'Finalise',
                                            'parsed':'Finalise'},
                               remapping={'error_code':'error_code',
                                          'error_message':'error_message'})
        
        smach.StateMachine.add('Finalise',
                               Finalise(),
                               remapping={'error_message':'error_message',
                                          'feedback':'feedback',
                                          'result':'result'},
                               transitions={'success':'success',
                                            'error':'error'})
    
    asw = ActionServerWrapper('move_gripper',
                              MoveGripperAction,
                              wrapped_container = sm,
                              goal_key = 'goal',
                              feedback_key = 'feedback',
                              result_key = 'result',
                              succeeded_outcomes = ['success'],
                              aborted_outcomes = ['error'],
                              preempted_outcomes = ['preempted'])
    
    asw.run_server()
    
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    