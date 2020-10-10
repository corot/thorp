#!/usr/bin/env python

import rospy
import smach
import smach_ros

from thorp_smach.toolkit.comon_states import ExecuteUserCommand
from thorp_smach.toolkit.manipulation_states import FoldArm, PickupObject, PlaceObject, ObjectDetection

import std_srvs.srv as std_srvs
import thorp_msgs.msg as thorp_msgs
import arbotix_msgs.srv as arbotix_srvs


rospy.init_node('object_manipulation_smach')

# Object manipulation top-level sm
sm = smach.StateMachine(outcomes=['stop', 'error', 'aborted', 'preempted'],
                        input_keys=['user_command'], output_keys=['ucmd_progress', 'ucmd_outcome'])
with sm:
    ''' User data at startup '''
    sm.userdata.user_command = thorp_msgs.UserCommandGoal()
    sm.userdata.ucmd_progress = thorp_msgs.UserCommandFeedback()
    sm.userdata.ucmd_outcome = thorp_msgs.UserCommandResult()
    sm.userdata.od_attempt = 0
    sm.userdata.output_frame = rospy.get_param('~rec_objects_frame', 'map')
    sm.userdata.pick_effort = 0.3
    #     sm.userdata.object_names   = []
    #     sm.userdata.object_name    = std_msg.String()
    #     sm.userdata.pick_pose      = geometry_msg.PoseStamped()
    #     sm.userdata.place_pose     = geometry_msg.PoseStamped()
    #     sm.userdata.named_pose_target_type = thorp_msgs.MoveToTargetGoal.NAMED_TARGET
    #     sm.userdata.arm_folded_named_pose = 'resting'
    #     sm.userdata.close_gripper  = control_msg.GripperCommand()
    #     sm.userdata.close_gripper.position = 0.0
    #     sm.userdata.open_gripper   = control_msg.GripperCommand()
    #     sm.userdata.open_gripper.position = 0.05

    smach.StateMachine.add('ExecuteUserCommand',
                           ExecuteUserCommand(['start', 'stop', 'reset', 'fold']),
                           transitions={'start': 'ObjectDetection',
                                        'reset': 'ObjectDetection',
                                        'fold': 'FoldArm',
                                        'stop': 'FoldArmAndRelax',
                                        'invalid_command': 'error'})

    smach.StateMachine.add('ObjectDetection',
                           ObjectDetection(),
                           remapping={'output_frame': 'output_frame',
                                      'object_names': 'object_names',
                                      'support_surf': 'support_surf'},
                           transitions={'succeeded': 'DragAndDrop',
                                        'preempted': 'preempted',
                                        'aborted': 'error'})

    smach.StateMachine.add('DragAndDrop',
                           smach_ros.SimpleActionState('drag_and_drop',
                                                       thorp_msgs.DragAndDropAction,
                                                       goal_slots=['object_names', 'output_frame'],
                                                       result_slots=['object_name', 'pick_pose', 'place_pose']),
                           remapping={'object_names': 'object_names',
                                      'object_name': 'object_name',
                                      'pick_pose': 'pick_pose',
                                      'place_pose': 'place_pose'},
                           transitions={'succeeded': 'PickupObject',
                                        'preempted': 'preempted',
                                        'aborted': 'ObjectDetection'})

    smach.StateMachine.add('PickupObject',
                           PickupObject(),
                           remapping={'object_name': 'object_name',
                                      'support_surf': 'support_surf',
                                      'max_effort': 'pick_effort'},
                           transitions={'succeeded': 'PlaceObject',
                                        'preempted': 'preempted',
                                        'aborted': 'ObjectDetection'})

    smach.StateMachine.add('PlaceObject',
                           PlaceObject(),
                           remapping={'object_name': 'object_name',
                                      'place_pose': 'place_pose'},
                           transitions={'succeeded': 'ObjectDetection',
                                        'preempted': 'preempted',
                                        'aborted': 'CLEAR_GRIPPER'})

    smach.StateMachine.add('CLEAR_GRIPPER',
                           smach_ros.ServiceState('clear_gripper', std_srvs.Empty),
                           transitions={'succeeded': 'ObjectDetection',
                                        'preempted': 'preempted',
                                        'aborted': 'aborted'})

    smach.StateMachine.add('FoldArm',
                           FoldArm(),
                           transitions={'succeeded': 'ObjectDetection',
                                        'preempted': 'preempted',
                                        'aborted': 'ObjectDetection'})

    smach.StateMachine.add('FoldArmAndRelax',
                           FoldArm(),
                           transitions={'succeeded': 'RelaxArmAndStop',
                                        'preempted': 'preempted',
                                        'aborted': 'error'})

    smach.StateMachine.add('RelaxArmAndStop',
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
