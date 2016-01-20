#!/usr/bin/env python

import rospy
import smach
import smach_ros

import std_msgs.msg as std_msg
import std_srvs.srv as std_srv
import thorp_msgs.msg as thorp_msg
import geometry_msgs.msg as geometry_msg

from actionlib import *
from actionlib_msgs.msg import *
from turtlebot_arm_object_manipulation.msg import *


class ObjDetectedCondition(smach.State):
    '''Check for the object detection result to retry if no objects where detected'''
    def __init__(self):
        ''' '''
        smach.State.__init__(self, outcomes=['preempted', 'satisfied', 'fold_arm', 'retry'],
                                   input_keys=['od_attempt', 'obj_names'],
                                   output_keys=['od_attempt'])

    def execute(self, userdata):
        rospy.sleep(2.0)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if len(userdata.obj_names) > 0:
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
        smach.State.__init__(self, outcomes=['start', 'reset', 'fold', 'quit'],
                                   input_keys=['user_command'])

    def execute(self, ud):
        rospy.loginfo("Executing User Command '%s'", ud['user_command'].command)
        return ud['user_command'].command


rospy.init_node('object_manipulation_smach')

# Object manipulation level sm; must be global so the concurrent USER_CMDS sm can signal to it the
# commands received by keyboard.  TODO: there must be a better way to do this!!!
sm = smach.StateMachine(outcomes=['quit', 'error', 'aborted', 'preempted'],
                        input_keys = ['user_command'], output_keys = ['ucmd_outcome'])
with sm:
    ''' User data '''
    sm.userdata.user_command   = thorp_msg.SmachCtrlGoal()
    sm.userdata.ucmd_outcome   = thorp_msg.SmachCtrlResult()
    sm.userdata.od_attempt     = 0
    sm.userdata.frame          = rospy.get_param('~arm_link', '/arm_link')
    sm.userdata.obj_names      = []
    sm.userdata.obj_name       = std_msg.String()
    sm.userdata.header         = std_msg.Header()
    sm.userdata.pick_pose      = geometry_msg.Pose()
    sm.userdata.place_pose     = geometry_msg.Pose()
    sm.userdata.named_pose_target_type = MoveToTargetGoal.NAMED_TARGET
    sm.userdata.arm_folded_named_pose = 'resting'

    smach.StateMachine.add('ExecuteUserCommand',
                           ExecuteUserCommand(),
                           transitions={'start':'ObjectDetection', 
                                        'reset':'ObjectDetection', 
                                        'fold':'FoldArm', 
                                        'quit':'FoldArmAndQuit'})

    # Object detection sub state machine; iterates over object_detection action state and recovery
    # mechanism until an object is detected, it's preempted or there's an error (aborted outcome)
    od_sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'],
                               input_keys = ['od_attempt', 'frame', 'obj_names',
                                             'named_pose_target_type', 'arm_folded_named_pose'],
                               output_keys = ['obj_names'])
    with od_sm:
        smach.StateMachine.add('ObjDetectionClearOctomap',
                               smach_ros.ServiceState('clear_octomap',
                                                      std_srv.Empty),
                               transitions={'succeeded':'ObjectDetectionOnce',
                                            'preempted':'preempted',
                                            'aborted':'ObjectDetectionOnce'})

        smach.StateMachine.add('ObjectDetectionOnce',
                               smach_ros.SimpleActionState('object_detection',
                                                           ObjectDetectionAction,
                                                           goal_slots=['frame'],
                                                           result_slots=['obj_names']),
                               remapping={'frame':'frame',
                                          'obj_names':'obj_names'},
                               transitions={'succeeded':'ObjDetectedCondition',
                                            'preempted':'preempted',
                                            'aborted':'aborted'})
        
        smach.StateMachine.add('ObjDetectedCondition',
                               ObjDetectedCondition(),
                               remapping={'obj_names':'obj_names'},
                               transitions={'satisfied':'succeeded',
                                            'preempted':'preempted',
                                            'fold_arm':'ObjDetectionFoldArm',
                                            'retry':'ObjDetectionClearOctomap'})

        smach.StateMachine.add('ObjDetectionFoldArm',
                               smach_ros.SimpleActionState('move_to_target',
                                                           MoveToTargetAction,
                                                           goal_slots=['target_type', 'named_target']),
                               remapping={'target_type':'named_pose_target_type',
                                          'named_target':'arm_folded_named_pose'},
                               transitions={'succeeded':'ObjDetectionClearOctomap',
                                            'preempted':'preempted',
                                            'aborted':'ObjDetectionClearOctomap'})

    smach.StateMachine.add('ObjectDetection', od_sm,
                           remapping={'frame':'frame'},
                           transitions={'succeeded':'InteractiveManip',
                                        'preempted':'preempted',
                                        'aborted':'error'})

    smach.StateMachine.add('InteractiveManip',
                           smach_ros.SimpleActionState('interactive_manipulation',
                                                       InteractiveManipAction,
                                                       goal_slots=['frame'],
                                                       result_slots=['obj_name', 'header', 'pick_pose', 'place_pose']),
                           remapping={'frame':'frame',
                                      'obj_name':'obj_name',
                                      'header':'header',
                                      'pick_pose':'pick_pose',
                                      'place_pose':'place_pose'},
                           transitions={'succeeded':'PickAndPlace',
                                        'preempted':'preempted',
                                        'aborted':'ObjectDetection'})

    smach.StateMachine.add('PickAndPlace',
                           smach_ros.SimpleActionState('pick_and_place',
                                                       PickAndPlaceAction,
                                                       goal_slots=['frame', 'obj_name', 'header', 'pick_pose', 'place_pose'],
                                                       result_slots=[]),
                           remapping={'frame':'frame',
                                      'header':'header',
                                      'pick_pose':'pick_pose',
                                      'place_pose':'place_pose'},
                           transitions={'succeeded':'ObjectDetection',
                                        'preempted':'preempted',
                                        'aborted':'ObjectDetection'}) # back to the beginning... we should open the gripper, in case we have picked an object (TODO)

    smach.StateMachine.add('FoldArm',
                           smach_ros.SimpleActionState('move_to_target',
                                                       MoveToTargetAction,
                                                       goal_slots=['target_type', 'named_target']),
                           remapping={'target_type':'named_pose_target_type',
                                      'named_target':'arm_folded_named_pose'},
                           transitions={'succeeded':'ObjectDetection',
                                        'preempted':'preempted',
                                        'aborted':'ObjectDetection'})

    smach.StateMachine.add('FoldArmAndQuit',
                           smach_ros.SimpleActionState('move_to_target',
                                                       MoveToTargetAction,
                                                       goal_slots=['target_type', 'named_target']),
                           remapping={'target_type':'named_pose_target_type',
                                      'named_target':'arm_folded_named_pose'},
                           transitions={'succeeded':'quit',
                                        'preempted':'quit',
                                        'aborted':'error'})


    # Construct action server wrapper
    asw = smach_ros.ActionServerWrapper(
        'user_commands_action_server', thorp_msg.SmachCtrlAction,
        wrapped_container = sm,
        succeeded_outcomes = ['quit'],
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
        
#     sm = smach.Concurrence(outcomes=['quit', 'error',],
#                            default_outcome='error',
#                            outcome_map={'quit' : {'OBJ_MANIP':'quit'},
#                                         'error' : {'OBJ_MANIP':'error'}},
#                                         child_termination_cb=child_term_cb)
# 
#     with sm:
#         smach.set_shutdown_check(rospy.is_shutdown)
#         smach.Concurrence.add('OBJ_MANIP', om_sm)
#         smach.Concurrence.add('USER_CMDS', 
#                                smach_ros.MonitorState("object_manipulation_keyboard/keydown", 
#                                                       keyboard_msgs.Key, 
#                                                       keydown_cb,
#                                                       output_keys=['user_command']))

#     # Create and start the introspection server
#     sis = smach_ros.IntrospectionServer('object_manipulation', sm, '/SM_ROOT')
#     sis.start()
    
#     # Execute the state machine
#     rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   ABOUT TO EXEC")
#     
#     # Create a thread to execute the smach container
#     smach_thread = threading.Thread(target=sm.execute)
#     smach_thread.start()
# 
#     # Wait for ctrl-c to stop the application
#     rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   ABOUT TO SPIN")
#     rospy.spin()
#    
#     # Request the container to preempt
#     rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   REQUEST PREEMPT")
#     sm.request_preempt()
#     
#     rospy.signal_shutdown('All done.')
#     
#     # Block until everything is preempted (we ignore execution outcome by now)
#     smach_thread.join()
# 
#     
#     rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   SPINNING!!!")
#     sis.stop()
# 
#     rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   STOPPED!!!!!!!!!!!")
# #     om_sm.request_preempt()
# #     rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   REQUEST PREEMPT")
#     
#     rospy.signal_shutdown('All done.')
#     rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   ALL DONE")


# if __name__ == '__main__':
#     main()
