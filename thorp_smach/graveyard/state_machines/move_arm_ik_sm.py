#!/usr/bin/env python
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
                                     input_keys=['goal_link_name',
                                                 'goal_pose',
                                                 'arm_joint_names',
                                                 'result'],
                                     output_keys=['result'])
    with sm_move_arm:
        sm_move_arm.userdata.zero_true = True
        sm_move_arm.userdata.zero_false = False
        sm_move_arm.userdata.default_true = True
        sm_move_arm.userdata.default_false = False
        sm_move_arm.userdata.bottom_true = True
        sm_move_arm.userdata.bottom_false = False
        sm_move_arm.userdata.robot_seed_state = moveit_msgs.msg.RobotState()
        sm_move_arm.userdata.robot_goal_state = moveit_msgs.msg.RobotState()
        sm_move_arm.userdata.avoid_collisions = True
        compute_ik_topic = "compute_ik"

        smach.StateMachine.add('GetRobotState',
                               misc_tools.GetRobotState(),
                               remapping={'robot_state':'robot_seed_state'},
                               transitions={'success':'PrepareIKSeedCurrentState'})

        smach.StateMachine.add('PrepareIKSeedCurrentState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_false',
                                          'default':'default_false',
                                          'bottom':'bottom_true',
                                          'robot_state':'robot_seed_state'},
                               transitions={'success':'GetIK',
                                            'error':'error'})

        smach.StateMachine.add('PrepareIKSeedZeroState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_true',
                                          'default':'default_false',
                                          'bottom':'bottom_true',
                                          'robot_state':'robot_seed_state'},
                               transitions={'success':'GetIK',
                                            'error':'error'})

        smach.StateMachine.add('PrepareIKSeedDefaultState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_false',
                                          'default':'default_true',
                                          'bottom':'bottom_true',
                                          'robot_state':'robot_seed_state'},
                               transitions={'success':'GetIK',
                                            'error':'error'})

        smach.StateMachine.add('PrepareIKSeedBottomState',
                               ik.PrepareIKSeed(),
                               remapping={'zero':'zero_false',
                                          'default':'default_false',
                                          'bottom':'bottom_true',
                                          'robot_state':'robot_seed_state'},
                               transitions={'success':'GetIK',
                                            'error':'error'})

        smach.StateMachine.add('GetIK',
                               ServiceState(compute_ik_topic,
                                            GetPositionIK,
                                            request_cb=ik.getPositionIKRequestCb,
                                            response_cb=ik.getPositionIKResponseRobotStateCb,
                                            input_keys=['goal_pose',
                                                        'goal_link_name',
                                                        'robot_seed_state',
                                                        'avoid_collisions'],
                                            output_keys=['robot_goal_state',
                                                         'error_code']),
                       remapping={'goal_pose':'goal_pose',
                                  'goal_link_name':'goal_link_name',
                                  'grasp_dist':'grasp_dist',
                                  'grasp_height':'grasp_height',
                                  'robot_seed_state':'robot_seed_state',
                                  'avoid_collisions':'avoid_collisions',
                                  'robot_goal_state':'robot_goal_state',
                                  'error_code':'error_code'},
                       transitions={'succeeded':'MoveArm',
                                    'preempted':'preempted',
                                    'aborted':'error',
                                    'failed':'TryAgain'})

        smach.StateMachine.add('TryAgain',
                               TryAgain(),
                               transitions={'zero_state':'PrepareIKSeedZeroState',
                                            'default_state':'PrepareIKSeedDefaultState',
                                            'bottom_state':'PrepareIKSeedBottomState',
                                            'done':'error',
                                            'error':'error'})

        smach.StateMachine.add('MoveArm',
                               SimpleActionState('move_group',
                                                 moveit_msgs.msg.MoveGroupAction,
                                                 goal_cb=move_arm.moveArmJointGoalCB,
                                                 result_cb=move_arm.moveArmResultCB,
                                                 input_keys=['robot_goal_state'],
                                                 output_keys=['error_code']),
                               remapping={'robot_goal_state':'robot_goal_state',
                                          'arm_joint_names':'arm_joint_names',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveArmErrorCode',
                                            'aborted':'ParseMoveArmErrorCode',
                                            'preempted':'preempted'})

        smach.StateMachine.add('ParseMoveArmErrorCode',
                               misc_tools.MoveItErrorCodesParser(),
                               transitions={'success':'success',
                                            'parsed':'error',
                                            'no_ik_solution':'error'},
                               remapping={'result':'result',
                                          'error_code':'error_code'})

        return sm_move_arm
