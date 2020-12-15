#!/usr/bin/env python

###
# Simplified interface for moving Korus' arm using MoveIt's motion planning capability (IK + joint space planning)
###

# system
import sys
import math
# ros basics & smach
import roslib; roslib.load_manifest('thorp_smach')
from thorp_smach.state_machines.state_machines_imports import *
from thorp_smach.state_machines import move_arm_ik_sm
from thorp_smach.pick_and_place_tools.msg_imports import *
from thorp_smach.pick_and_place_tools import move_arm, trajectory_control, misc_tools


class ParseGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=['incoming_goal'],
                             output_keys=['feedback',
                                          'goal_pose'])

    def execute(self, userdata):
        feedback = MoveArmFeedback()
        feedback.process_state = 'Finished parsing MoveArmGoal'
        userdata.feedback = feedback
        userdata.goal_pose = userdata.incoming_goal.goal_pose
        return 'success'


def main():
    rospy.init_node('move_arm_planner_as')

    sm = smach.StateMachine(outcomes=['success',
                                      'preempted',
                                      'error'],
                            input_keys=['goal'],
                            output_keys=['feedback',
                                         'result'])

    sm.userdata.goal_link_name = "palm_link"
    sm.userdata.motors = ['torso_turn', 'torso_lift', 'shoulder', 'elbow', 'wrist']
    sm.userdata.arm_joint_names = ['torso_turn', 'torso_lift', 'shoulder', 'elbow', 'wrist']
    sm.userdata.feedback = MoveArmFeedback()
    sm.userdata.result = MoveArmResult()
    sm.userdata.error_code = int()
    
    with sm:
        smach.StateMachine.add('ParseGoal',
                               ParseGoal(),
                               remapping={'incoming_goal':'goal',
                                          'feedback':'feedback',
                                          'goal_pose':'goal_pose'},
                               transitions={'success':'EnableMotors'})
        
        smach.StateMachine.add('EnableMotors',
                               misc_tools.EnableMotors(),
                               transitions={'success':'MoveArmIK'},
                               remapping={'motors':'motors'})
        
        sm_move_arm_ik = move_arm_ik_sm.createSM()
        smach.StateMachine.add('MoveArmIK',
                               sm_move_arm_ik,
                               remapping={'goal_link_name':'goal_link_name',
                                          'goal_pose':'goal_pose',
                                          'arm_joint_names':'arm_joint_names',
                                          'result':'result'},
                               transitions={'success':'success',
                                            'preempted':'preempted',
                                            'error':'error'})
        
    asw = ActionServerWrapper('move_arm_planner',
                              MoveArmAction,
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

