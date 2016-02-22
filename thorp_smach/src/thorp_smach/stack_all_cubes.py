#!/usr/bin/env python

import math
import rospy
import smach
import smach_ros

import thorp_toolkit as ttk

import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import moveit_msgs.msg as moveit_msgs
import control_msgs.msg as control_msgs
import arbotix_msgs.srv as arbotix_srvs
import geometry_msgs.msg as geometry_msgs

from actionlib import *
from actionlib_msgs.msg import *


class GetDetectedCubes(smach.State):
    ''' Check for the object detection result to extract only the cubes and select one as the stack base '''
    def __init__(self):
        ''' '''
        smach.State.__init__(self, outcomes=['succeeded', 'retry'],
                                   input_keys=['objects', 'object_names', 'arm_ref_frame'],
                                   output_keys=['base_cube_pose', 'base_cube_name', 'other_cubes'])

    def execute(self, userdata):
        # Compose a list containing id, pose, distance and heading for all cubes within arm reach
        objects = []
        for obj in userdata.objects:
            if not obj.id.startswith('cube'):
                continue
            obj_pose = ttk.get_pose_from_co(obj, stamped=True)
            obj_pose = ttk.transform_pose(userdata.arm_ref_frame, obj_pose)
            distance = ttk.distance_2d(obj_pose.pose)
            if distance > 0.3:
                rospy.logdebug("'%s' is out of reach (%d > %d)",  obj.id, distance, 0.3)
                continue
            heading = ttk.heading(obj_pose.pose)
            objects.append((obj.id, obj_pose, distance, heading))
        # Check if we have at least 2 cubes to stack 
        if len(objects) < 2:
            return 'retry'
        # Sort them by increasing heading; we stack over the one most in front of the arm, called base
        objects = sorted(objects, key=lambda x: abs(x[-1]))
        userdata.base_cube_name = objects[0][0]
        userdata.base_cube_pose = objects[0][1]
        userdata.other_cubes = [str(obj[0]) for obj in objects[1:]]
        return 'succeeded'

class ObjDetectedCondition(smach.State):
    ''' Check for the object detection result to retry if no objects where detected '''
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


class IncreasePlaceHeight(smach.State):
    ''' Increase place pose height by ud.cube_height '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                   input_keys=['base_cube_pose', 'cube_height'],
                                   output_keys=['base_cube_pose'])

    def execute(self, ud):
        ud.base_cube_pose.pose.position.z += ud.cube_height
        return 'succeeded'


class ExecuteUserCommand(smach.State):
    ''' Different starts of the SM depending on the command provided when calling
        the actionlib wrapper. TODO: I think this can be done w/o creating a class... '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['start', 'stop', 'reset', 'fold'],
                                   input_keys=['user_command'])

    def execute(self, ud):
        rospy.loginfo("Executing User Command '%s'", ud['user_command'].command)
        return ud['user_command'].command


rospy.init_node('stack_all_cubes_smach')

# Object manipulation top-level sm
sm = smach.StateMachine(outcomes=['stop', 'error', 'aborted', 'preempted'],
                        input_keys = ['user_command'], output_keys = ['ucmd_outcome'])
with sm:
    ''' User data at startup '''
    sm.userdata.user_command   = thorp_msgs.UserCommandGoal()
    sm.userdata.ucmd_outcome   = thorp_msgs.UserCommandResult()
    sm.userdata.od_attempt     = 0
    sm.userdata.arm_ref_frame  = rospy.get_param('~arm_ctrl_ref_frame', 'arm_base_link')
    sm.userdata.output_frame   = rospy.get_param('~rec_objects_frame', 'map')
    sm.userdata.cube_height    = 0.025  # TODO get from Collision object!
    sm.userdata.named_pose_target_type = thorp_msgs.MoveToTargetGoal.NAMED_TARGET
    sm.userdata.arm_folded_named_pose = 'resting'
    sm.userdata.close_gripper  = control_msgs.GripperCommand()
    sm.userdata.close_gripper.position = 0.0
    sm.userdata.open_gripper   = control_msgs.GripperCommand()
    sm.userdata.open_gripper.position = 0.05
    
    # Other fields created at runtime are objects, object_names, object_name, base_cube_name, base_cube_pose and other_cubes


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
                                                           control_msgs.GripperCommandAction,
                                                           goal_slots=['command']),
                               remapping={'command':'close_gripper'})
        smach.Concurrence.add('FoldArm',
                               smach_ros.SimpleActionState('move_to_target',
                                                           thorp_msgs.MoveToTargetAction,
                                                           goal_slots=['target_type', 'named_target']),
                               remapping={'target_type':'named_pose_target_type',
                                          'named_target':'arm_folded_named_pose'})

    # Object detection sub state machine; iterates over object_detection action state and recovery
    # mechanism until an object is detected, it's preempted or there's an error (aborted outcome)
    od_sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'],
                               input_keys = ['od_attempt', 'output_frame','close_gripper',
                                             'named_pose_target_type', 'arm_folded_named_pose'],
                               output_keys = ['objects', 'object_names'])
    with od_sm:
        smach.StateMachine.add('ClearOctomap',
                               smach_ros.ServiceState('clear_octomap',
                                                      std_srvs.Empty),
                               transitions={'succeeded':'ObjectDetection',
                                            'preempted':'preempted',
                                            'aborted':'ObjectDetection'})

        smach.StateMachine.add('ObjectDetection',
                               smach_ros.SimpleActionState('object_detection',
                                                           thorp_msgs.DetectObjectsAction,
                                                           goal_slots=['output_frame'],
                                                           result_slots=['objects', 'object_names']),
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
                           transitions={'succeeded':'GetDetectedCubes',
                                        'preempted':'preempted',
                                        'aborted':'error'})

        
    smach.StateMachine.add('GetDetectedCubes',
                           GetDetectedCubes(),
                           remapping={'object_names':'object_names'},
                           transitions={'succeeded':'StackCubes',
                                        'retry':'ObjectDetection'})

    # Stack cubes sub state machine; iterates over the detected cubes and stack them over the one most in front of the arm
    sc_it = smach.Iterator(outcomes = ['succeeded','preempted','aborted'],
                           input_keys = ['base_cube_pose', 'base_cube_name', 'other_cubes', 'cube_height'],
                           output_keys = [],
                           it = lambda: sm.userdata.other_cubes,  # must be a lambda because we destroy the list
                           it_label = 'object_name',
                           exhausted_outcome = 'succeeded')
    with sc_it:
        sc_sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted','continue'],
                                   input_keys = ['base_cube_pose', 'object_name', 'other_cubes', 'cube_height'],
                                   output_keys = [])
        with sc_sm:
            smach.StateMachine.add('PickupObject',
                                   smach_ros.SimpleActionState('pickup_object',
                                                               thorp_msgs.PickupObjectAction,
                                                               goal_slots=['object_name'],
                                                               result_slots=[]),
                                   remapping={'object_name':'object_name'},
                                   transitions={'succeeded':'IncreasePlaceHeight',
                                                'preempted':'preempted',
                                                'aborted':'aborted'}) # back to the beginning... we should open the gripper, in case we have picked an object (TODO)

            smach.StateMachine.add('IncreasePlaceHeight',
                                   IncreasePlaceHeight(),
                                   transitions={'succeeded':'PlaceObject'})

            smach.StateMachine.add('PlaceObject',
                                   smach_ros.SimpleActionState('place_object',
                                                               thorp_msgs.PlaceObjectAction,
                                                               goal_slots=['object_name', 'place_pose'],
                                                               result_slots=[]),
                                   remapping={'object_name':'object_name',
                                              'place_pose':'base_cube_pose'},
                                   transitions={'succeeded':'continue',
                                                'preempted':'preempted',
                                                'aborted':'aborted'}) # back to the beginning... we should open the gripper, in case we have picked an object (TODO)

        smach.Iterator.set_contained_state('', sc_sm, loop_outcomes=['continue'])

    smach.StateMachine.add('StackCubes', sc_it,
                 {'succeeded':'FoldArmAndRelax',
                  'aborted':'error'})

    smach.StateMachine.add('FoldArm', fa_cc,
                           transitions={'succeeded':'ObjectDetection',
                                        'preempted':'preempted',
                                        'aborted':'ObjectDetection'})

    smach.StateMachine.add('FoldArmAndRelax',
                           smach_ros.SimpleActionState('move_to_target',
                                                       thorp_msgs.MoveToTargetAction,
                                                       goal_slots=['target_type', 'named_target']),
                           remapping={'target_type':'named_pose_target_type',
                                      'named_target':'arm_folded_named_pose'},
                           transitions={'succeeded':'RelaxArmAndStop',
                                        'preempted':'preempted',
                                        'aborted':'error'})

    smach.StateMachine.add('RelaxArmAndStop',
                           smach_ros.ServiceState('servos/relax_all',
                                                  arbotix_srvs.Relax),
                           transitions={'succeeded':'stop',
                                        'preempted':'stop',
                                        'aborted':'error'})


    # Construct action server wrapper for top-level sm to control it with keyboard commands
    asw = smach_ros.ActionServerWrapper(
        'user_commands_action_server', thorp_msgs.UserCommandAction,
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
