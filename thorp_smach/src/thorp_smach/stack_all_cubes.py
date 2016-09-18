#!/usr/bin/env python

import math
import rospy
import smach
import smach_ros

import thorp_toolkit as ttk
from thorp_smach.toolkit.comon_states import *

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import control_msgs.msg as control_msgs
import arbotix_msgs.srv as arbotix_srvs

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
            # Object's timestamp is irrelevant, and can trigger a TransformException if very recent; zero it!
            obj_pose = ttk.get_pose_from_co(obj, stamped=True)
            obj_pose.header.stamp = rospy.Time(0.0)
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


class IncreasePlaceHeight(smach.State):
    ''' Increase place pose height by ud.cube_height '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                   input_keys=['base_cube_pose', 'cube_height'],
                                   output_keys=['base_cube_pose'])

    def execute(self, ud):
        ud.base_cube_pose.pose.position.z += ud.cube_height
        return 'succeeded'


rospy.init_node('stack_all_cubes_smach')

# Object manipulation top-level sm
sm = smach.StateMachine(outcomes=['stop', 'error', 'aborted', 'preempted'],
                        input_keys = ['user_command'], output_keys = ['ucmd_progress', 'ucmd_outcome'])
with sm:
    ''' User data at startup '''
    sm.userdata.user_command   = thorp_msgs.UserCommandGoal()
    sm.userdata.ucmd_progress  = thorp_msgs.UserCommandFeedback()
    sm.userdata.ucmd_outcome   = thorp_msgs.UserCommandResult()
    sm.userdata.od_attempt     = 0
    sm.userdata.arm_ref_frame  = rospy.get_param('~arm_ctrl_ref_frame', 'arm_base_link')
    sm.userdata.output_frame   = rospy.get_param('~rec_objects_frame', 'map')
    sm.userdata.cube_height    = 0.025  # TODO get from Collision object!
#     sm.userdata.named_pose_target_type = thorp_msgs.MoveToTargetGoal.NAMED_TARGET
#     sm.userdata.arm_folded_named_pose = 'resting'
#     sm.userdata.close_gripper  = control_msgs.GripperCommand()
#     sm.userdata.close_gripper.position = 0.0
#     sm.userdata.open_gripper   = control_msgs.GripperCommand()
#     sm.userdata.open_gripper.position = 0.05
    
    # Other fields created at runtime are objects, object_names, object_name, base_cube_name, base_cube_pose and other_cubes


    smach.StateMachine.add('ExecuteUserCommand',
                           ExecuteUserCommand(['start', 'stop', 'fold']),
                           transitions={'start':'ObjectDetection',
                                        'fold':'FoldArm', 
                                        'stop':'FoldArmAndRelax',
                                        'invalid_command':'error'})

    smach.StateMachine.add('ObjectDetection',
                           ObjectDetection(),
                           remapping={'output_frame':'output_frame',
                                      'object_names':'object_names',
                                      'support_surf':'support_surf'},
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
                           input_keys = ['base_cube_pose', 'base_cube_name', 'other_cubes', 'cube_height', 'support_surf'],
                           output_keys = [],
                           it = lambda: sm.userdata.other_cubes,  # must be a lambda because we destroy the list
                           it_label = 'object_name',
                           exhausted_outcome = 'succeeded')
    with sc_it:
        sc_sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted','continue'],
                                   input_keys = ['base_cube_pose', 'object_name', 'support_surf', 'cube_height'],
                                   output_keys = [])
        with sc_sm:
            smach.StateMachine.add('PickupObject',
                                   PickupObject(),
                                   remapping={'object_name':'object_name',
                                              'support_surf':'support_surf'},
                                   transitions={'succeeded':'IncreasePlaceHeight',
                                                'preempted':'preempted',
                                                'aborted':'aborted'})

            smach.StateMachine.add('IncreasePlaceHeight',
                                   IncreasePlaceHeight(),
                                   transitions={'succeeded':'PlaceObject'})

            smach.StateMachine.add('PlaceObject',
                                   PlaceObject(),
                                   remapping={'object_name':'object_name',
                                              'support_surf':'support_surf',
                                              'place_pose':'base_cube_pose'},
                                   transitions={'succeeded':'continue',
                                                'preempted':'preempted',
                                                'aborted':'aborted'})

        smach.Iterator.set_contained_state('', sc_sm, loop_outcomes=['continue'])

    smach.StateMachine.add('StackCubes', sc_it,
                 {'succeeded':'FoldArmAndRelax',
                  'aborted':'error'})

    smach.StateMachine.add('FoldArm',
                           FoldArm(),
                           transitions={'succeeded':'ObjectDetection',
                                        'preempted':'preempted',
                                        'aborted':'ObjectDetection'})

    smach.StateMachine.add('FoldArmAndRelax',
                           FoldArm(),
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
    asw = smach_ros.ActionServerWrapper('user_commands_action_server',
                                        thorp_msgs.UserCommandAction,
                                        wrapped_container = sm,
                                        succeeded_outcomes = ['stop'],
                                        aborted_outcomes = ['aborted'],
                                        preempted_outcomes = ['error'],
                                        goal_key = 'user_command',
                                        feedback_key = 'ucmd_progress',
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
