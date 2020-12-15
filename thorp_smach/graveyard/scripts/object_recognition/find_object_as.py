#!/usr/bin/env python

###
# Simplified interface for finding objects around Korus
#
# Points the head to specific position and does object recognition 
###

# system
import sys
import math
# ros basics& smach
import rospy
import smach
from smach import StateMachine
import smach_ros
from smach_ros import ActionServerWrapper
import tf
from tf import TransformListener

import geometry_msgs
import control_msgs

from thorp_smach.state_machines import find_object_sm
#from thorp_smach.pick_and_place_tools import object_recognition
from pick_and_place_msgs.msg import FindObjectAction
from pick_and_place_msgs.msg import FindObjectFeedback
from pick_and_place_msgs.msg import FindObjectResult
import object_recognition_msgs


class Prepare(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['prepared'],
                             input_keys=['goal',
                                         'min_confidence',
                                         'feedback'],
                             output_keys=['table_pose',
                                          'look_around',
                                          'min_confidence',
                                          'feedback'])
    def execute(self, userdata):
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = userdata.goal.table_position.header.frame_id
        table_pose.header.stamp = userdata.goal.table_position.header.stamp
        table_pose.pose.position.x = userdata.goal.table_position.point.x 
        table_pose.pose.position.y = userdata.goal.table_position.point.y 
        table_pose.pose.position.z = userdata.goal.table_position.point.z
        table_pose.pose.orientation.x = 0.0
        table_pose.pose.orientation.y = 0.0
        table_pose.pose.orientation.z = 0.0
        table_pose.pose.orientation.w = 1.0
        userdata.table_pose = table_pose
        userdata.look_around = userdata.goal.look_around
        if not userdata.goal.min_confidence == 0.0:
             userdata.min_confidence = userdata.goal.min_confidence
        print userdata.min_confidence
        userdata.feedback.process_state = "FindObject input prepared."
        return 'prepared'

class Finalise(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['prepared'],
                             input_keys=['recognised_objects',
                                         'object_names',
                                         'error_message',
                                         'error_code',
                                         'result',
                                         'feedback',
                                         'tf_listener'],
                             output_keys=['feedback',
                                          'result',
                                          'tf_listener'])
        
    def execute(self, userdata):
        result = FindObjectResult()
        for object in userdata.recognised_objects.objects:
            object_pose = geometry_msgs.msg.PoseStamped()
            object_pose.header = object.pose.header
            object_pose.pose = object.pose.pose.pose
            result.object_poses.append(object_pose)
            result.object_confidences.append(object.confidence)
            result.object_types.append(str(object.type.key))
            print userdata.tf_listener.transformPose("base_footprint", object_pose)
        result.object_names = userdata.object_names
        result.error_message = userdata.error_message
        result.error_code = userdata.error_code
        userdata.result = result
        userdata.feedback.process_state = "FindObject result prepared."
        return 'prepared'


def main():
    rospy.init_node('find_object')
    
    sm = StateMachine(outcomes=['success',
                                'aborted',
                                'preempted'],
                      input_keys=['goal'],
                      output_keys=['result'])
    with sm:
        sm.userdata.feedback = FindObjectFeedback()
        sm.userdata.result = FindObjectResult()
        sm.userdata.min_confidence = 0.5
        sm.userdata.recognised_objects = object_recognition_msgs.msg.RecognizedObjectArray()
        sm.userdata.object_names = list()
#        sm.userdata.objects_info = list()
        sm.userdata.error_code = int()
        sm.userdata.error_message = str()
        
        sm.userdata.tf_listener = tf.TransformListener()
        
        smach.StateMachine.add('Prepare',
                               Prepare(),
                               remapping={'goal':'goal',
                                          'min_confidence':'min_confidence',
                                          'table_pose':'table_pose',
                                          'look_around':'look_around',
                                          'feedback':'feedback',
                                          'tf_listener':'tf_listener'},
                               transitions={'prepared':'FindObject'})
        
        sm_find_object = find_object_sm.createSM()
        smach.StateMachine.add('FindObject',
                               sm_find_object,
                               remapping={'table_pose':'table_pose',
                                          'look_around':'look_around',
                                          'min_confidence':'min_confidence',
                                          'recognised_objects':'recognised_objects',
                                          'object_names':'object_names',
                                          'error_message':'error_message',
                                          'error_code':'error_code',
                                          'tf_listener':'tf_listener'},
                               transitions={'object_found':'Finalise',
                                            'no_objects_found':'Finalise',
                                            'aborted':'aborted',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('Finalise',
                               Finalise(),
                               remapping={'recognised_objects':'recognised_objects',
                                          'object_names':'object_names',
                                          'error_message':'error_message',
                                          'error_code':'error_code',
                                          'result':'result',
                                          'feedback':'feedback',
                                          'result':'result'},
                               transitions={'prepared':'success'})
    
    asw = ActionServerWrapper('find_object',
                              FindObjectAction,
                              wrapped_container = sm,
                              goal_key = 'goal',
                              feedback_key = 'feedback',
                              result_key = 'result',
                              succeeded_outcomes = ['success'],
                              aborted_outcomes = ['aborted'],
                              preempted_outcomes = ['preempted'])
    
    asw.run_server()
    
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
