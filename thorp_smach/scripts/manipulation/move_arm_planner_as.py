#!/usr/bin/env python

###
# Simplified interface for moving Korus' arm using MoveIt's motion planning capability (task space planning)
###

# system
import sys
import math
# ros basics & smach
from thorp_smach.state_machines.state_machines_imports import *
from thorp_smach.state_machines import move_arm_sm
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
        feedback = pick_and_place_msgs.MoveArmFeedback()
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
    sm.userdata.feedback = pick_and_place_msgs.MoveArmFeedback()
    sm.userdata.result = pick_and_place_msgs.MoveArmResult()
    sm.userdata.error_code = int()
    sm.userdata.pose_goal = True
        
    with sm:
        smach.StateMachine.add('ParseGoal',
                               ParseGoal(),
                               remapping={'incoming_goal':'goal',
                                          'feedback':'feedback',
                                          'goal_pose':'goal_pose'},
                               transitions={'success':'EnableMotors'})
        
        smach.StateMachine.add('EnableMotors',
                               misc_tools.EnableMotors(),
                               transitions={'success':'MoveArm'},
                               remapping={'motors':'motors'})
        
        sm_move_arm = move_arm_sm.createSM()
        smach.StateMachine.add('MoveArm',
                               sm_move_arm,
                               remapping={'goal_link_name':'goal_link_name',
                                          'goal_pose':'goal_pose',
                                          'result':'result'},
                               transitions={'success':'success',
                                            'preempted':'preempted',
                                            'error':'error'})
        
    asw = ActionServerWrapper('move_arm_planner',
                              pick_and_place_msgs.MoveArmAction,
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

