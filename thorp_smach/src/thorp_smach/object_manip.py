#!/usr/bin/env python

import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import arbotix_msgs.srv as arbotix_srvs

from toolkit.common_states import ExecuteUserCommand
from toolkit.perception_states import ObjectDetectionSM
from toolkit.manipulation_states import FoldArm, PickupObject, PlaceObject


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
    sm.userdata.output_frame = rospy.get_param('~rec_objects_frame', 'map')  # ignored by RAIL
    sm.userdata.max_effort = 0.03

    smach.StateMachine.add('EXE_USER_CMD',
                           ExecuteUserCommand(rospy.get_param('object_manip_key_ctrl/valid_commands')),
                           transitions={'start': 'OBJECT_DETECTION',
                                        'reset': 'OBJECT_DETECTION',
                                        'clear': 'CLEAR_GRIPPER',
                                        'fold': 'FOLD_ARM',
                                        'stop': 'FOLD_ARM_AND_RELAX',
                                        'invalid_command': 'error'})

    smach.StateMachine.add('OBJECT_DETECTION',
                           ObjectDetectionSM(),
                           transitions={'succeeded': 'DRAG_AND_DROP',
                                        'preempted': 'preempted',
                                        'aborted': 'error'})

    smach.StateMachine.add('DRAG_AND_DROP',
                           smach_ros.SimpleActionState('drag_and_drop',
                                                       thorp_msgs.DragAndDropAction,
                                                       goal_slots=['object_names', 'output_frame'],
                                                       result_slots=['object_name', 'pick_pose', 'place_pose']),
                           transitions={'succeeded': 'PICKUP_OBJECT',
                                        'preempted': 'preempted',
                                        'aborted': 'OBJECT_DETECTION'})

    smach.StateMachine.add('PICKUP_OBJECT',
                           PickupObject(),
                           transitions={'succeeded': 'PLACE_OBJECT',
                                        'preempted': 'preempted',
                                        'aborted': 'OBJECT_DETECTION'})

    smach.StateMachine.add('PLACE_OBJECT',
                           PlaceObject(),
                           transitions={'succeeded': 'OBJECT_DETECTION',
                                        'preempted': 'preempted',
                                        'aborted': 'CLEAR_GRIPPER'})

    smach.StateMachine.add('CLEAR_GRIPPER',
                           smach_ros.ServiceState('clear_gripper', std_srvs.Empty),
                           transitions={'succeeded': 'DRAG_AND_DROP',
                                        'preempted': 'preempted',
                                        'aborted': 'aborted'})

    smach.StateMachine.add('FOLD_ARM',
                           FoldArm(),
                           transitions={'succeeded': 'OBJECT_DETECTION',
                                        'preempted': 'preempted',
                                        'aborted': 'OBJECT_DETECTION'})

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
