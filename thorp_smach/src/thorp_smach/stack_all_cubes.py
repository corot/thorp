#!/usr/bin/env python

import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import arbotix_msgs.srv as arbotix_srvs
import geometry_msgs.msg as geometry_msgs
import thorp_msgs.msg as thorp_msgs

from thorp_toolkit.geometry import TF2, get_size_from_co, distance_2d, heading

from thorp_smach.states.common import ExecuteUserCommand
from thorp_smach.states.geometry import TranslatePose
from thorp_smach.states.perception import DetectObjects
from thorp_smach.states.manipulation import FoldArm, PickupObject, PlaceObject, DisplaceObject

from thorp_smach.utils import run_sm


class GetDetectedCubes(smach.State):
    """ Check for the object detection result to extract only the cubes and select one as the stack base """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'retry'],
                             input_keys=['objects', 'arm_ref_frame'],
                             output_keys=['place_pose', 'other_cubes'])

    def execute(self, ud):
        # Compose a list containing id, pose, distance and heading for all cubes within arm reach
        max_arm_reach = rospy.get_param('~max_arm_reach')
        objects = []
        for obj in ud.objects:
            if not obj.id.startswith('cube'):
                continue
            # Object's timestamp is irrelevant, and can trigger a TransformException if very recent; zero it!
            obj_pose = geometry_msgs.PoseStamped(obj.header, obj.pose)
            obj_pose.header.stamp = rospy.Time(0)
            obj_pose = TF2().transform_pose(obj_pose, obj_pose.header.frame_id, ud.arm_ref_frame)
            distance = distance_2d(obj_pose.pose)
            if distance > max_arm_reach:
                rospy.logdebug("'%s' is out of reach (%d > %d)", obj.id, distance, max_arm_reach)
                continue
            direction = heading(obj_pose.pose)
            objects.append((obj, obj_pose, distance, direction))
        # Check if we have at least 2 cubes to stack 
        if len(objects) < 2:
            return 'retry'
        # Sort them by increasing heading; we stack over the one most in front of the arm, called base
        objects = sorted(objects, key=lambda x: abs(x[-1]))
        place_pose = objects[0][1]
        place_pose.pose.position.z += get_size_from_co(objects[0][0])[2] + rospy.get_param('~placing_height_on_table')
        ud.place_pose = place_pose
        ud.other_cubes = [obj[0] for obj in objects[1:]]
        return 'succeeded'


class IncreasePlaceHeight(smach.State):
    """ Increase place pose height by last stacked object's z-dimension """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['place_pose', 'object'],
                             output_keys=['place_pose'])

    def execute(self, ud):
        ud.place_pose.pose.position.z += get_size_from_co(ud.object)[2] + rospy.get_param('~placing_height_on_table')
        return 'succeeded'


rospy.init_node('stack_all_cubes_smach')

# Object manipulation top-level sm
sm = smach.StateMachine(outcomes=['stop', 'error', 'aborted', 'preempted'],
                        input_keys=['user_command'], output_keys=['ucmd_progress', 'ucmd_outcome'])
with sm:
    """ User data at startup """
    sm.userdata.user_command = thorp_msgs.UserCommandGoal()
    sm.userdata.ucmd_progress = thorp_msgs.UserCommandFeedback()
    sm.userdata.ucmd_outcome = thorp_msgs.UserCommandResult()
    sm.userdata.od_attempt = 0
    sm.userdata.arm_ref_frame = rospy.get_param('~arm_ctrl_ref_frame', 'arm_base_link')

    smach.StateMachine.add('EXE_USER_CMD',
                           ExecuteUserCommand(['start', 'stop', 'fold', 'clear']),
                           transitions={'start': 'DETECT_OBJECTS',
                                        'fold': 'FOLD_ARM',
                                        'clear': 'CLEAR_GRIPPER',
                                        'stop': 'FOLD_ARM_AND_RELAX',
                                        'invalid_command': 'error'})
    smach.StateMachine.add('DETECT_OBJECTS',
                           DetectObjects(),
                           transitions={'succeeded': 'GET_DETECTED_CUBES',
                                        'preempted': 'preempted',
                                        'aborted': 'error'})
    smach.StateMachine.add('GET_DETECTED_CUBES',
                           GetDetectedCubes(),
                           transitions={'succeeded': 'STACK_CUBES',
                                        'retry': 'DETECT_OBJECTS'})

    # Stack cubes sub state machine; iterates over the detected cubes and stack them over the one most in front of the arm
    sc_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                           input_keys=['place_pose', 'other_cubes', 'surface'],
                           output_keys=[],
                           it=lambda: sm.userdata.other_cubes,  # must be a lambda because we destroy the list
                           it_label='object',
                           exhausted_outcome='succeeded')
    with sc_it:
        sc_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                   input_keys=['place_pose', 'object', 'surface'],
                                   output_keys=[])
        sc_sm.userdata.max_effort = rospy.get_param('~gripper_max_effort')
        sc_sm.userdata.tightening = sc_sm.userdata.max_effort / 2.0
        with sc_sm:
            smach.StateMachine.add('PICKUP_OBJECT',
                                   PickupObject(),
                                   transitions={'succeeded': 'PLACE_OBJECT',
                                                'preempted': 'preempted',
                                                'aborted': 'continue'})
            smach.StateMachine.add('PLACE_OBJECT',
                                   PlaceObject(),
                                   transitions={'succeeded': 'AT_STACK_LEVEL',
                                                'preempted': 'preempted',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('AT_STACK_LEVEL', TranslatePose(-rospy.get_param('~placing_height_on_table'), 'z'),
                                   remapping={'pose': 'place_pose'},             # undo added clearance
                                   transitions={'succeeded': 'READJUST_POSE'})   # to replicate gravity
            smach.StateMachine.add('READJUST_POSE',
                                   DisplaceObject(),
                                   remapping={'new_pose': 'place_pose'},
                                   transitions={'succeeded': 'INC_PLACE_HEIGHT'})
            smach.StateMachine.add('INC_PLACE_HEIGHT',
                                   IncreasePlaceHeight(),
                                   transitions={'succeeded': 'continue'})

        smach.Iterator.set_contained_state('', sc_sm, loop_outcomes=['continue'])

    smach.StateMachine.add('STACK_CUBES', sc_it,
                           {'succeeded': 'FOLD_ARM_AND_RELAX',
                            'aborted': 'error'})
    smach.StateMachine.add('CLEAR_GRIPPER',
                           smach_ros.ServiceState('clear_gripper', std_srvs.Empty),
                           transitions={'succeeded': 'DETECT_OBJECTS',
                                        'preempted': 'preempted',
                                        'aborted': 'aborted'})
    smach.StateMachine.add('FOLD_ARM',
                           FoldArm(),
                           transitions={'succeeded': 'DETECT_OBJECTS',
                                        'preempted': 'preempted',
                                        'aborted': 'DETECT_OBJECTS'})
    smach.StateMachine.add('FOLD_ARM_AND_RELAX',
                           FoldArm(),
                           transitions={'succeeded': 'RELAX_ARM_AND_STOP',
                                        'preempted': 'preempted',
                                        'aborted': 'error'})
    smach.StateMachine.add('RELAX_ARM_AND_STOP',
                           smach_ros.ServiceState('servos/relax_all',
                                                  arbotix_srvs.Relax),
                           transitions={'succeeded': 'stop',
                                        'preempted': 'stop',
                                        'aborted': 'error'})

# Construct action server wrapper for top-level sm to control it with keyboard commands
asw = smach_ros.ActionServerWrapper('user_commands_action_server',
                                    thorp_msgs.UserCommandAction,
                                    wrapped_container=sm,
                                    succeeded_outcomes=['stop'],
                                    aborted_outcomes=['aborted'],
                                    preempted_outcomes=['error'],
                                    goal_key='user_command',
                                    feedback_key='ucmd_progress',
                                    result_key='ucmd_outcome')

run_sm(sm, rospy.get_param('~app_name'), asw)
