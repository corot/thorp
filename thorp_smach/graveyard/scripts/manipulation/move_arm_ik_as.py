#!/usr/bin/env python

###
# Simplified interface for moving Korus' arm using IK and joint trajectory interpolation only 
###

# system
import sys
import math
# ros basics & smach
from thorp_smach.state_machines.state_machines_imports import *
from thorp_smach.pick_and_place_tools.msg_imports import *
from thorp_smach.pick_and_place_tools import ik, trajectory_control, misc_tools

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
    rospy.init_node('move_arm_ik_as')
    
    compute_ik_topic = "compute_ik"
    
    sm = smach.StateMachine(outcomes=['success',
                                      'preempted',
                                      'error'],
                            input_keys=['goal'],
                            output_keys=['feedback',
                                         'result'])
    
    sm.userdata.robot_state = moveit_msgs.RobotState()
    sm.userdata.goal_link_name = "palm_link"
    sm.userdata.grasp_dist = 0.0
    sm.userdata.grasp_height = 0.0
    sm.userdata.result = pick_and_place_msgs.MoveArmResult()
    sm.userdata.error_code = int()
    sm.userdata.motors = ['torso_turn', 'torso_lift', 'shoulder', 'elbow', 'wrist']
    sm.userdata.zero_true = True
    sm.userdata.zero_false = False 
    sm.userdata.default_true = True
    sm.userdata.default_false = False 
    sm.userdata.bottom_true = True
    sm.userdata.bottom_false = False
    sm.userdata.false = False
    
    with sm:
        smach.StateMachine.add('ParseGoal',
                               ParseGoal(),
                               remapping={'incoming_goal':'goal',
                                          'feedback':'feedback',
                                          'goal_pose':'goal_pose'},
                               transitions={'success':'GetRobotState'})
        
        smach.StateMachine.add('GetRobotState',
                               misc_tools.GetRobotState(),
                               remapping={'robot_state':'robot_state'},
                               transitions={'success':'PrepareIKSeedcurrentState'})
        
        smach.StateMachine.add('PrepareIKSeedcurrentState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_false',
                                          'default':'default_false',
                                          'bottom':'bottom_false',
                                          'robot_state':'robot_state'},
                               transitions={'success':'GetIKcurrentState',
                                            'error':'error'})
        
        smach.StateMachine.add('GetIKcurrentState',
                               ServiceState(compute_ik_topic,
                                            moveit_srvs.GetPositionIK,
                                            request_cb = ik.getPositionIKRequestCb,
                                            response_cb = ik.getPositionIKResponseArmControllerGoalCb,
                                            input_keys=['goal_pose',
                                                        'goal_link_name',
                                                        'robot_state',
                                                        'avoid_collisions'],
                                            output_keys=['arm_control_goal',
                                                         'error_code']),
                               remapping={'goal_pose':'goal_pose',
                                          'goal_link_name':'goal_link_name',
                                          'robot_state':'robot_state',
                                          'avoid_collisions':'false',
                                          'arm_control_goal':'arm_control_goal',
                                          'error_code':'error_code',
                                          'error_message':'error_message'},
                               transitions={'succeeded':'ParseIKcurrentStateErrorCode',
                                            'preempted':'preempted',
                                            'aborted':'error',
                                            'failed':'ParseIKcurrentStateErrorCode'})
        
        smach.StateMachine.add('ParseIKcurrentStateErrorCode',
                               misc_tools.MoveItErrorCodesParser(),
                               transitions={'success':'EnableMotors',
                                            'no_ik_solution':'error',
                                            'planning_failed':'error',
                                            'parsed':'error'},
                               remapping={'result':'result',
                                          'error_code':'error_code'})
        
        smach.StateMachine.add('PrepareIKSeedzeroState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_true',
                                          'default':'default_false',
                                          'bottom':'bottom_false',
                                          'robot_state':'robot_state'},
                               transitions={'success':'GetIKzeroState',
                                            'error':'error'})
        
        smach.StateMachine.add('GetIKzeroState',
                               ServiceState(compute_ik_topic,
                                            moveit_srvs.GetPositionIK,
                                            request_cb = ik.getPositionIKRequestCb,
                                            response_cb = ik.getPositionIKResponseArmControllerGoalCb,
                                            input_keys=['goal_pose',
                                                        'goal_link_name',
                                                        'robot_state',
                                                        'avoid_collisions'],
                                            output_keys=['arm_control_goal',
                                                         'error_code']),
                               remapping={'goal_pose':'goal_pose',
                                          'goal_link_name':'goal_link_name',
                                          'grasp_dist':'grasp_dist',
                                          'grasp_height':'grasp_height',
                                          'robot_state':'robot_state',
                                          'avoid_collisions':'false',
                                          'arm_control_goal':'arm_control_goal',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseIKzeroStateErrorCode',
                                            'preempted':'preempted',
                                            'aborted':'error',
                                            'failed':'ParseIKzeroStateErrorCode'})
        
        smach.StateMachine.add('ParseIKzeroStateErrorCode',
                               misc_tools.MoveItErrorCodesParser(),
                               transitions={'success':'EnableMotors',
                                            'no_ik_solution':'PrepareIKSeedDefaultState',
                                            'planning_failed':'error',
                                            'parsed':'error'},
                               remapping={'result':'result',
                                          'error_code':'error_code'})
        
        smach.StateMachine.add('PrepareIKSeedDefaultState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_false',
                                          'default':'default_true',
                                          'bottom':'bottom_false',
                                          'robot_state':'robot_state'},
                               transitions={'success':'GetIKdefaultState',
                                            'error':'error'})
        
        smach.StateMachine.add('GetIKdefaultState',
                               ServiceState(compute_ik_topic,
                                            moveit_srvs.GetPositionIK,
                                            request_cb = ik.getPositionIKRequestCb,
                                            response_cb = ik.getPositionIKResponseArmControllerGoalCb,
                                            input_keys=['goal_pose',
                                                        'goal_link_name',
                                                        'robot_state',
                                                        'avoid_collisions'],
                                            output_keys=['arm_control_goal',
                                                         'error_code']),
                               remapping={'goal_pose':'goal_pose',
                                          'goal_link_name':'goal_link_name',
                                          'grasp_dist':'grasp_dist',
                                          'grasp_height':'grasp_height',
                                          'robot_state':'robot_state',
                                          'avoid_collisions':'false',
                                          'arm_control_goal':'arm_control_goal',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseIKdefaultStateErrorCode',
                                            'preempted':'preempted',
                                            'aborted':'error',
                                            'failed':'ParseIKdefaultStateErrorCode'})
        
        smach.StateMachine.add('ParseIKdefaultStateErrorCode',
                               misc_tools.MoveItErrorCodesParser(),
                               transitions={'success':'EnableMotors',
                                            'no_ik_solution':'PrepareIKSeedBottomState',
                                            'planning_failed':'error',
                                            'parsed':'error'},
                               remapping={'result':'result',
                                          'error_code':'error_code'})
        
        smach.StateMachine.add('PrepareIKSeedBottomState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_false',
                                          'default':'default_false',
                                          'bottom':'bottom_true',
                                          'robot_state':'robot_state'},
                               transitions={'success':'PrepareIKSeedBottomState',
                                            'error':'error'})
        
        smach.StateMachine.add('GetIKBottomState',
                               ServiceState(compute_ik_topic,
                                            moveit_srvs.GetPositionIK,
                                            request_cb = ik.getPositionIKRequestCb,
                                            response_cb = ik.getPositionIKResponseArmControllerGoalCb,
                                            input_keys=['goal_pose',
                                                        'goal_link_name',
                                                        'robot_state',
                                                        'avoid_collisions'],
                                            output_keys=['arm_control_goal',
                                                         'error_code']),
                               remapping={'goal_pose':'goal_pose',
                                          'goal_link_name':'goal_link_name',
                                          'grasp_dist':'grasp_dist',
                                          'grasp_height':'grasp_height',
                                          'robot_state':'robot_state',
                                          'avoid_collisions':'false',
                                          'arm_control_goal':'arm_control_goal',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseIKBottomStateErrorCode',
                                            'preempted':'preempted',
                                            'aborted':'error',
                                            'failed':'ParseIKBottomStateErrorCode'})
        
        smach.StateMachine.add('ParseIKBottomStateErrorCode',
                               misc_tools.MoveItErrorCodesParser(),
                               transitions={'success':'EnableMotors',
                                            'no_ik_solution':'error',
                                            'planning_failed':'error',
                                            'parsed':'error'},
                               remapping={'result':'result',
                                          'error_code':'error_code'})
        
        smach.StateMachine.add('EnableMotors',
                               misc_tools.EnableMotors(),
                               transitions={'success':'MoveArm'},
                               remapping={'motors':'motors'})
        
        smach.StateMachine.add('MoveArm',
                               SimpleActionState('arm_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.generalGoalCb,
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 output_keys=['error_code'],),
                               remapping={'trajectory_goal':'arm_control_goal',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveArmErrorCode',
                                            'aborted':'ParseMoveArmErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('ParseMoveArmErrorCode',
                               trajectory_control.FollowJointTrajectoryErrorCodesParser(),
                               transitions={'success':'success',
                                            'parsed':'error'},
                               remapping={'result':'result',
                                          'error_code':'error_code'})
    
    asw = ActionServerWrapper('move_arm_ik',
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
    