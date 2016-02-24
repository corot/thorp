#!/usr/bin/env python

import roslib; roslib.load_manifest('thorp_smach')
from state_machines_imports import *
from thorp_smach.pick_and_place_tools import trajectory_control, move_arm, ik, misc_tools
from thorp_smach.pick_and_place_tools.msg_imports import *


class TryAgain(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['zero_state',
                                       'default_state',
                                       'bottom_state',
                                       'done',
                                       'error'])
        self._state = 0

    def execute(self, userdata):
        if self._state is -1:
            rospy.loginfo("All available seed states have been tested.")
            self._state = 0
            return 'done'
        elif self._state is 0:
            self._state = 1
            return 'zero_state'
        elif self._state is 1:
            self._state = 2
            return 'default_state'
        elif self._state is 2:
            self._state = -1
            return 'bottom_state'
        else:
            rospy.logerr("Don't know this state (" + str(self._state) + ")")
            self._state = 0
            return 'error'

def createSM():
    sm_move_arm = smach.StateMachine(outcomes=['success', 'preempted', 'error'],
                                     input_keys=['goal_link_name','goal_pose', 'result'],
                                     output_keys=['result'])
    with sm_move_arm:
        sm_move_arm.userdata.default_pose = geometry_msgs.msg.PoseStamped()
        sm_move_arm.userdata.default_pose.header.stamp = rospy.Time.now()
        sm_move_arm.userdata.default_pose.header.frame_id = "base_footprint"
        sm_move_arm.userdata.default_pose.pose.position.x = 0.40#0.052
        sm_move_arm.userdata.default_pose.pose.position.y = 0.0
        sm_move_arm.userdata.default_pose.pose.position.z = 0.50 #0.412
        sm_move_arm.userdata.default_pose.pose.orientation.x = 0.7071
        sm_move_arm.userdata.default_pose.pose.orientation.y = 0.0032
        sm_move_arm.userdata.default_pose.pose.orientation.z = -0.0032
        sm_move_arm.userdata.default_pose.pose.orientation.w = 0.7071
        sm_move_arm.userdata.wait_0sec = 0.0
        sm_move_arm.userdata.wait_1sec = 1.0
        sm_move_arm.userdata.wait_2sec = 2.0
        sm_move_arm.userdata.wait_5sec = 5.0
        sm_move_arm.userdata.zero_true = True
        sm_move_arm.userdata.zero_false = False 
        sm_move_arm.userdata.default_true = True
        sm_move_arm.userdata.default_false = False 
        sm_move_arm.userdata.bottom_true = True
        sm_move_arm.userdata.bottom_false = False 
        sm_move_arm.userdata.robot_state = moveit_msgs.msg.RobotState()
        sm_move_arm.userdata.avoid_collisions = True

#        smach.StateMachine.add('ResetCollisionMap',
#                               ServiceState('/korus/octomap_server/reset',
#                                            std_srvs.srv.Empty()),
#                                            transitions={'succeeded':'MoveArm',
#                                                         'preempted':'preempted',
#                                                         'aborted':'error'})
        smach.StateMachine.add('GetRobotState',
                               misc_tools.GetRobotState(),
                               remapping={'robot_state':'robot_state'},
                               transitions={'success':'PrepareIKSeedCurrentState'})
        
        smach.StateMachine.add('PrepareIKSeedCurrentState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_false',
                                          'default':'default_false',
                                          'bottom':'bottom_true',
                                          'robot_state':'robot_state'},
                               transitions={'success':'MoveArm',
                                            'error':'error'})
        
        smach.StateMachine.add('PrepareIKSeedZeroState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_true',
                                          'default':'default_false',
                                          'bottom':'bottom_true',
                                          'robot_state':'robot_state'},
                               transitions={'success':'MoveArm',
                                            'error':'error'})
        
        smach.StateMachine.add('PrepareIKSeedDefaultState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_false',
                                          'default':'default_true',
                                          'bottom':'bottom_true',
                                          'robot_state':'robot_state'},
                               transitions={'success':'MoveArm',
                                            'error':'error'})
        
        smach.StateMachine.add('PrepareIKSeedBottomState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_false',
                                          'default':'default_false',
                                          'bottom':'bottom_true',
                                          'robot_state':'robot_state'},
                               transitions={'success':'MoveArm',
                                            'error':'error'})
        
        smach.StateMachine.add('GetIK',
                               ServiceState(compute_ik_topic,
                                            GetPositionIK,
                                            request_cb = ik.getPositionIKRequestCb,
                                            response_cb = ik.getPositionIKResponseCb,
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
                                  'arm_control_goal':'arm_control_goal',
                                  'error_code':'error_code'},
                       transitions={'succeeded':'ParseIKdefaultStateErrorCode',
                                    'preempted':'preempted',
                                    'aborted':'error',
                                    'failed':'ParseIKdefaultStateErrorCode'})
        
        smach.StateMachine.add('MoveArm',
                               SimpleActionState('move_group',
                                                 moveit_msgs.msg.MoveGroupAction,
                                                 goal_cb=move_arm.moveArmGoalCB,
                                                 result_cb=move_arm.moveArmResultCB,
                                                 input_keys=['goal_pose',
                                                             'goal_link_name',
                                                             'robot_state'],
                                                 output_keys=['error_code']),
                               remapping={'goal_pose':'goal_pose',
                                          'goal_link_name':'goal_link_name',
                                          'robot_state':'robot_state',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveArmErrorCode',
                                            'aborted':'ParseMoveArmErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('ParseMoveArmErrorCode',
                               misc_tools.MoveItErrorCodesParser(),
                               transitions={'success':'success',
                                            'parsed':'error',
                                            'no_ik_solution':'TryAgain'},
                               remapping={'result':'result',
                                          'error_code':'error_code'})
        
        smach.StateMachine.add('TryAgain',
                               TryAgain(),
                               transitions={'zero_state':'PrepareIKSeedZeroState',
                                            'default_state':'PrepareIKSeedDefaultState',
                                            'bottom_state':'PrepareIKSeedBottomState',
                                            'done':'error',
                                            'error':'error'})
        return sm_prepare_ik_seeds