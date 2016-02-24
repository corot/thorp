#!/usr/bin/env python

import rospy
import smach
import smach_ros

import std_srvs.srv as std_srvs
import keyboard.msg as keyboard_msgs
import geometry_msgs.msg as geometry_msgs
import move_base_msgs.msg as move_base_msgs

from actionlib import *
from actionlib_msgs.msg import *
from turtlebot_arm_object_manipulation.msg import *


USER_COMMANDS = {
    keyboard_msgs.Key.KEY_s: "start",
    keyboard_msgs.Key.KEY_r: "reset",
    keyboard_msgs.Key.KEY_f: "fold",
    keyboard_msgs.Key.KEY_q: "quit"
}

app_started = False

def started_cb(ud):
    return app_started
    
def keydown_cb(ud, msg):
    rospy.loginfo("Key pressed: %s", msg.code)
    if msg.code == keyboard_msgs.Key.KEY_s:
        global app_started
        app_started = True
    elif msg.code in USER_COMMANDS:
        om_sm.userdata['user_command'] = USER_COMMANDS[msg.code]
        om_sm.request_preempt()
   
    return True

def child_term_cb(states):
    rospy.loginfo("Concurrency child terminated: %s", states)
    if None not in states.values():
        rospy.loginfo("All concurrency children terminated; shutdown executive SMACH")
        rospy.signal_shutdown("All concurrency children done")

    # Preempt all other states 
    return True


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

class WaitCondition(smach.State):
    '''Wait until a condition is satisfied. 
    When the condition is satisfied, the value that matched the condition is stored in the userdata.
    The callback must return that value or something that evaluates to False otherwise. 
    The arguments to the callback are userdata, robot'''
    def __init__(self, condition_callback, timeout):
        ''' ''' 
        smach.State.__init__(self,
                             outcomes=['satisfied', 'timed_out', 'preempted'],
                             output_keys=['trigger_value'])
        self.condition_callback = condition_callback
        self.timeout = timeout

    def execute(self, userdata): 
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time) < rospy.Duration(self.timeout) \
          and not rospy.is_shutdown():
            cb_output = self.condition_callback(userdata)
            if cb_output:
                userdata.trigger_value = cb_output
                return 'satisfied'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)
        app_started = True
        return 'timed_out'

class ActionPreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reset', 'fold', 'quit'],
                                   input_keys=['user_command'])

    def execute(self, ud):
        rospy.loginfo("Executing User Command '%s'", ud['user_command'])
        return ud['user_command']

# Object manipulation level sm; must be global so the concurrent USER_CMDS sm can signal to it the
# commands received by keyboard.  TODO: there must be a better way to do this!!!
om_sm = smach.StateMachine(outcomes=['quit', 'error'])

def main():
    rospy.init_node('object_manipulation_smach')

    with om_sm:
        ''' general '''
        om_sm.userdata.true = True
        om_sm.userdata.false = False
        ''' table poses '''
        om_sm.userdata.od_attempt     = 0
        om_sm.userdata.frame          = rospy.get_param('~arm_link', '/arm_link')
        om_sm.userdata.obj_names      = []
        om_sm.userdata.obj_name       = std_msgs.msg.String()
        om_sm.userdata.header         = std_msgs.msg.Header()
        om_sm.userdata.pick_pose      = geometry_msgs.msg.Pose()
        om_sm.userdata.place_pose     = geometry_msgs.msg.Pose()
        om_sm.userdata.named_pose_target_type = MoveToTargetGoal.NAMED_TARGET
        om_sm.userdata.arm_folded_named_pose = 'resting'

        smach.StateMachine.add('WaitForStartCmd',
                               WaitCondition(started_cb, 30.0),
                               transitions={'satisfied':'ObjectDetection', 
                                            'timed_out':'ObjectDetection', 
                                            'preempted':'ActionPreempted'})

        # Object detection sub state machine; iterates over object_detection action state and recovery
        # mechanism until an object is detected, it's preempted or there's an error (aborted outcome)
        od_sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'],
                                   input_keys = ['od_attempt', 'frame', 'obj_names',
                                                 'named_pose_target_type', 'arm_folded_named_pose'],
                                   output_keys = ['obj_names'])
        with od_sm:
            smach.StateMachine.add('ObjDetectionClearOctomap',
                                   smach_ros.ServiceState('clear_octomap',
                                                          std_srvs.Empty),
                                   transitions={'succeeded':'ObjectDetectionOnce',
                                                'preempted':'ObjectDetectionOnce',
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
                                            'preempted':'ActionPreempted',
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
                                            'preempted':'ActionPreempted',
                                            'aborted':'error'})

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
                                            'preempted':'ActionPreempted',
                                            'aborted':'ObjectDetection'}) # back to the beginning... we should open the gripper, in case we have picked an object (TODO)

        smach.StateMachine.add('FoldArm',
                               smach_ros.SimpleActionState('move_to_target',
                                                           MoveToTargetAction,
                                                           goal_slots=['target_type', 'named_target']),
                               remapping={'target_type':'named_pose_target_type',
                                          'named_target':'arm_folded_named_pose'},
                               transitions={'succeeded':'ObjectDetection',
                                            'preempted':'ActionPreempted',
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

        smach.StateMachine.add('ActionPreempted',
                               ActionPreempted(),
                               transitions={'reset':'ObjectDetection', 
                                            'fold':'FoldArm', 
                                            'quit':'FoldArmAndQuit'})

    sm = smach.Concurrence(outcomes=['quit', 'error',],
                           default_outcome='error',
                           outcome_map={'quit' : {'OBJ_MANIP':'quit'},
                                        'error' : {'OBJ_MANIP':'error'}},
                                        child_termination_cb=child_term_cb)

    with sm:
        smach.set_shutdown_check(rospy.is_shutdown)
        smach.Concurrence.add('OBJ_MANIP', om_sm)
        smach.Concurrence.add('USER_CMDS', 
                               smach_ros.MonitorState("object_manipulation_keyboard/keydown", 
                                                      keyboard_msgs.Key, 
                                                      keydown_cb,
                                                      output_keys=['user_command']))

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('object_manipulation', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   ABOUT TO EXEC")
    
    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c to stop the application
    rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   ABOUT TO SPIN")
    rospy.spin()
   
    # Request the container to preempt
    rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   REQUEST PREEMPT")
    sm.request_preempt()
    
    rospy.signal_shutdown('All done.')
    
    # Block until everything is preempted (we ignore execution outcome by now)
    smach_thread.join()

    
    rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   SPINNING!!!")
    sis.stop()

    rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   STOPPED!!!!!!!!!!!")
#     om_sm.request_preempt()
#     rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   REQUEST PREEMPT")
    
    rospy.signal_shutdown('All done.')
    rospy.logerr(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>   ALL DONE")


if __name__ == '__main__':
    main()
