#!/usr/bin/env python

import rospy
import smach
import smach_ros

import std_msgs.msg as std_msg
import std_srvs.srv as std_srv
import thorp_msgs.msg as thorp_msg
import control_msgs.msg as control_msg
import arbotix_msgs.srv as arbotix_srv
import geometry_msgs.msg as geometry_msg

from actionlib import *
from actionlib_msgs.msg import *


class ObjDetectedCondition(smach.State):
    '''Check for the object detection result to retry if no objects where detected'''
    def __init__(self):
        ''' '''
        smach.State.__init__(self, outcomes=['preempted', 'satisfied', 'fold_arm', 'retry'],
                                   input_keys=['od_attempt', 'object_names'],
                                   output_keys=['od_attempt'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if len(userdata.object_names) > 0:
            userdata.od_attempt = 0
            return 'satisfied'
        userdata.od_attempt += 1
        if userdata.od_attempt == 1:
            return 'fold_arm'
        return 'retry'

class ExecuteUserCommand(smach.State):
    '''Different starts of the SM depending on the command provided when calling
       the actionlib wrapper. TODO: I think this can be done w/o creating a class...'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['start', 'stop', 'reset', 'fold'],
                                   input_keys=['user_command'])

    def execute(self, ud):
        rospy.loginfo("Executing User Command '%s'", ud['user_command'].command)
        return ud['user_command'].command


rospy.init_node('object_manipulation_smach')

# Object manipulation top-level sm
sm = smach.StateMachine(outcomes=['stop', 'error', 'aborted', 'preempted'],
                        input_keys = ['user_command'], output_keys = ['ucmd_outcome'])
with sm:
    ''' User data '''
    sm.userdata.user_command   = thorp_msg.UserCommandGoal()
    sm.userdata.ucmd_outcome   = thorp_msg.UserCommandResult()
    sm.userdata.od_attempt     = 0
    sm.userdata.output_frame   = rospy.get_param('~rec_objects_frame', '/map')
    sm.userdata.object_names   = []
    sm.userdata.object_name    = std_msg.String()
    sm.userdata.pick_pose      = geometry_msg.PoseStamped()
    sm.userdata.place_pose     = geometry_msg.PoseStamped()
    sm.userdata.named_pose_target_type = thorp_msg.MoveToTargetGoal.NAMED_TARGET
    sm.userdata.arm_folded_named_pose = 'resting'
    sm.userdata.close_gripper  = control_msg.GripperCommand()
    sm.userdata.close_gripper.position = 0.0
    sm.userdata.open_gripper   = control_msg.GripperCommand()
    sm.userdata.open_gripper.position = 0.05

    smach.StateMachine.add('ExecuteUserCommand',
                           ExecuteUserCommand(),
                           transitions={'start':'ObjectDetection', 
                                        'reset':'ObjectDetection', 
                                        'fold':'FoldArm', 
                                        'stop':'FoldArmAndRelax'})

    # Concurrently fold arm and close the gripper
    fa_cc = smach.Concurrence(outcomes = ['succeeded', 'preempted', 'aborted'],
                              default_outcome = 'succeeded',
                              input_keys = ['close_gripper',
                                            'named_pose_target_type',
                                            'arm_folded_named_pose'],
                              outcome_map = {'succeeded':{'CloseGripper':'succeeded',
                                                          'FoldArm':'succeeded'},
                                             'preempted':{'CloseGripper':'preempted',
                                                          'FoldArm':'preempted'},
                                             'aborted':{'CloseGripper':'aborted',
                                                        'FoldArm':'aborted'}})
    with fa_cc:
        smach.Concurrence.add('CloseGripper',
                               smach_ros.SimpleActionState('gripper_controller/gripper_action',
                                                           control_msg.GripperCommandAction,
                                                           goal_slots=['command']),
                               remapping={'command':'close_gripper'})
        smach.Concurrence.add('FoldArm',
                               smach_ros.SimpleActionState('move_to_target',
                                                           thorp_msg.MoveToTargetAction,
                                                           goal_slots=['target_type', 'named_target']),
                               remapping={'target_type':'named_pose_target_type',
                                          'named_target':'arm_folded_named_pose'})

    # Object detection sub state machine; iterates over object_detection action state and recovery
    # mechanism until an object is detected, it's preempted or there's an error (aborted outcome)
    od_sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'],
                               input_keys = ['od_attempt', 'output_frame','close_gripper',
                                             'named_pose_target_type', 'arm_folded_named_pose'],
                               output_keys = ['object_names'])
    with od_sm:
        smach.StateMachine.add('ClearOctomap',
                               smach_ros.ServiceState('clear_octomap',
                                                      std_srv.Empty),
                               transitions={'succeeded':'ObjectDetection',
                                            'preempted':'preempted',
                                            'aborted':'ObjectDetection'})

        smach.StateMachine.add('ObjectDetection',
                               smach_ros.SimpleActionState('object_detection',
                                                           thorp_msg.DetectObjectsAction,
                                                           goal_slots=['output_frame'],
                                                           result_slots=['object_names']),
                               remapping={'output_frame':'output_frame',
                                          'object_names':'object_names'},
                               transitions={'succeeded':'ObjDetectedCondition',
                                            'preempted':'preempted',
                                            'aborted':'aborted'})
        
        smach.StateMachine.add('ObjDetectedCondition',
                               ObjDetectedCondition(),
                               remapping={'object_names':'object_names'},
                               transitions={'satisfied':'succeeded',
                                            'preempted':'preempted',
                                            'fold_arm':'FoldArm',
                                            'retry':'ClearOctomap'})

        smach.StateMachine.add('FoldArm', fa_cc,
                               transitions={'succeeded':'ClearOctomap',
                                            'preempted':'preempted',
                                            'aborted':'ClearOctomap'})

    smach.StateMachine.add('ObjectDetection', od_sm,
                           remapping={'output_frame':'output_frame',
                                      'object_names':'object_names'},
                           transitions={'succeeded':'InteractiveManip',
                                        'preempted':'preempted',
                                        'aborted':'error'})

    smach.StateMachine.add('InteractiveManip',
                           smach_ros.SimpleActionState('interactive_manipulation',
                                                       thorp_msg.InteractiveManipAction,
                                                       goal_slots=['object_names'],
                                                       result_slots=['object_name', 'pick_pose', 'place_pose']),
                           remapping={'object_names':'object_names',
                                      'object_name':'object_name',
                                      'pick_pose':'pick_pose',
                                      'place_pose':'place_pose'},
                           transitions={'succeeded':'PickupObject',
                                        'preempted':'preempted',
                                        'aborted':'ObjectDetection'})

    smach.StateMachine.add('PickupObject',
                           smach_ros.SimpleActionState('pickup_object',
                                                       thorp_msg.PickupObjectAction,
                                                       goal_slots=['object_name'],
                                                       result_slots=[]),
                           remapping={'object_name':'object_name'},
                           transitions={'succeeded':'PlaceObject',
                                        'preempted':'preempted',
                                        'aborted':'ObjectDetection'}) # back to the beginning... we should open the gripper, in case we have picked an object (TODO)

    smach.StateMachine.add('PlaceObject',
                           smach_ros.SimpleActionState('place_object',
                                                       thorp_msg.PlaceObjectAction,
                                                       goal_slots=['object_name', 'place_pose'],
                                                       result_slots=[]),
                           remapping={'object_name':'object_name',
                                      'place_pose':'place_pose'},
                           transitions={'succeeded':'ObjectDetection',
                                        'preempted':'preempted',
                                        'aborted':'ObjectDetection'}) # back to the beginning... we should open the gripper, in case we have picked an object (TODO)

    smach.StateMachine.add('FoldArm', fa_cc,
                           transitions={'succeeded':'ObjectDetection',
                                        'preempted':'preempted',
                                        'aborted':'ObjectDetection'})

    smach.StateMachine.add('FoldArmAndRelax',
                           smach_ros.SimpleActionState('move_to_target',
                                                       thorp_msg.MoveToTargetAction,
                                                       goal_slots=['target_type', 'named_target']),
                           remapping={'target_type':'named_pose_target_type',
                                      'named_target':'arm_folded_named_pose'},
                           transitions={'succeeded':'RelaxArmAndStop',
                                        'preempted':'preempted',
                                        'aborted':'error'})

    smach.StateMachine.add('RelaxArmAndStop',
                           smach_ros.ServiceState('servos/relax_all',
                                                  arbotix_srv.Relax),
                           transitions={'succeeded':'stop',
                                        'preempted':'stop',
                                        'aborted':'error'})


    # Construct action server wrapper for top-level sm to control it with keyboard commands
    asw = smach_ros.ActionServerWrapper(
        'user_commands_action_server', thorp_msg.UserCommandAction,
        wrapped_container = sm,
        succeeded_outcomes = ['stop'],
        aborted_outcomes = ['aborted'],
        preempted_outcomes = ['error'],
        goal_key = 'user_command',
        result_key = 'ucmd_outcome')
    
    # Run the server in a background thread
    asw.run_server()
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('object_manipulation', sm, '/SM_ROOT')
    sis.start()

    # Wait for control-c
    rospy.spin()

    rospy.loginfo("Stopping '%s' node...", rospy.get_name())
    sis.stop()

    rospy.signal_shutdown('All done.')
