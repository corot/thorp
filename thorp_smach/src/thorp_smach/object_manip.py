#!/usr/bin/env python

import rospy
import smach
import smach_ros

from actionlib import GoalStatus

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import arbotix_msgs.srv as arbotix_srvs

from thorp_smach.states.common import ExecuteUserCommand
from thorp_smach.states.perception import DetectObjects
from thorp_smach.states.manipulation import FoldArm, PickupObject, PlaceObject, ClearPlanningScene

from thorp_smach.utils import run_sm
from thorp_smach import config as cfg

rospy.init_node('object_manip_smach')

# Object manipulation top-level sm
sm = smach.StateMachine(outcomes=['stop', 'error', 'aborted', 'preempted'],
                        input_keys=['user_command'], output_keys=['ucmd_progress', 'ucmd_outcome'])
with sm:
    ''' User data at startup '''
    sm.userdata.user_command = thorp_msgs.UserCommandGoal()
    sm.userdata.ucmd_progress = thorp_msgs.UserCommandFeedback()
    sm.userdata.ucmd_outcome = thorp_msgs.UserCommandResult()
    sm.userdata.od_attempt = 0
    sm.userdata.output_frame = cfg.PICKING_PLANNING_FRAME
    sm.userdata.max_effort = cfg.GRIPPER_MAX_EFFORT
    sm.userdata.tightening = cfg.GRIPPER_TIGHTENING

    # drag_and_drop operates with object names, while object detection provides a list collision objects
    # and pick/drop actions target is also a collision object; hence, we need these conversion callbacks
    def list_obj_names(ud, goal):
        goal.object_names = [co.id for co in ud['objects']]
        return goal

    def get_selected_obj(ud, status, result):
        if status == GoalStatus.SUCCEEDED:
            selected_obj = next((obj for obj in ud['objects'] if obj.id == result.object_name), None)
            if selected_obj is None:
                # This should be impossible, as the manipulable object names list is created from the objects
                raise KeyError("Manipulated object '%s' not found in objects list")
            ud['object'] = selected_obj

    smach.StateMachine.add('EXE_USER_CMD',
                           ExecuteUserCommand(rospy.get_param('object_manip_user_commands/valid_commands')),
                           transitions={'start': 'DETECT_OBJECTS',
                                        'reset': 'DETECT_OBJECTS',
                                        'clear': 'CLEAR_GRIPPER',
                                        'fold': 'FOLD_ARM',
                                        'stop': 'FOLD_ARM_AND_RELAX',
                                        'invalid_command': 'error'})
    smach.StateMachine.add('DETECT_OBJECTS', DetectObjects(),
                           transitions={'succeeded': 'DRAG_AND_DROP',
                                        'preempted': 'preempted',
                                        'aborted': 'aborted'})
    smach.StateMachine.add('DRAG_AND_DROP',
                           smach_ros.SimpleActionState('drag_and_drop',
                                                       thorp_msgs.DragAndDropAction,
                                                       input_keys=['objects'],
                                                       output_keys=['object'],
                                                       goal_cb=list_obj_names,
                                                       goal_slots=['output_frame'],
                                                       result_cb=get_selected_obj,
                                                       result_slots=['pickup_pose', 'place_pose']),
                           transitions={'succeeded': 'PICKUP_OBJECT',
                                        'preempted': 'preempted',
                                        'aborted': 'DETECT_OBJECTS'})
    smach.StateMachine.add('PICKUP_OBJECT',
                           PickupObject(),
                           transitions={'succeeded': 'PLACE_OBJECT',
                                        'preempted': 'preempted',
                                        'aborted': 'DETECT_OBJECTS'})
    smach.StateMachine.add('PLACE_OBJECT',
                           PlaceObject(),
                           transitions={'succeeded': 'DETECT_OBJECTS',
                                        'preempted': 'preempted',
                                        'aborted': 'CLEAR_GRIPPER'})
    smach.StateMachine.add('CLEAR_GRIPPER',
                           smach_ros.ServiceState('clear_gripper', std_srvs.Empty),
                           transitions={'succeeded': 'DRAG_AND_DROP',
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
    smach.StateMachine.add('CLEAR_PLANNING_SCENE',
                           ClearPlanningScene(),
                           transitions={'succeeded': 'stop'})

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
