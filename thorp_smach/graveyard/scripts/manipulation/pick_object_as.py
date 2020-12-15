#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('thorp_smach')
import thorp_smach
from thorp_smach.state_machines import pick_object_sm
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from smach_ros import ActionServerWrapper
import tf

import geometry_msgs.msg as geometry_msgs
import manipulation_msgs.msg as manipulation_msgs
import moveit_msgs.msg as moveit_msgs
import pick_and_place_msgs.msg as pick_and_place_msgs
import sensor_msgs.msg as sensor_msgs


class ParseGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['parsed'],
                             input_keys=['goal'],
                             output_keys=['object_name',
                                          'object_pose',
                                          'feedback'])

    def execute(self, userdata):
        rospy.loginfo('Parsing pick object goal ...')
        userdata.object_name = userdata.goal.object_name
        object_pose = geometry_msgs.PoseStamped()
        object_pose = userdata.goal.object_pose
#        object_pose.pose = userdata.goal.object_pose.pose
#        object_pose.pose.position.x = userdata.goal.object_pose.pose.position.x - 0.15
#        object_pose.pose.position.z = userdata.goal.object_pose.pose.position.z + 0.10
        userdata.object_pose = object_pose
        feedback = pick_and_place_msgs.PickObjectFeedback()
        feedback.process_state = 'PickObjectGoal parsed'
        userdata.feedback = feedback
        rospy.loginfo('Gripper goal prepared.')
        return 'parsed'

def main():
    rospy.init_node('pick_object')
    
    sm = smach.StateMachine(outcomes=['picked',
                                      'pick_failed',
                                      'preempted',
                                      'error'],
                            input_keys=['goal'],
                            output_keys=['feedback',
                                         'result'])

    with sm:
        sm.userdata.pose_arm_default = geometry_msgs.PoseStamped()
        sm.userdata.pose_arm_default.header.stamp = rospy.Time.now()
        sm.userdata.pose_arm_default.header.frame_id = "/base_footprint"
        sm.userdata.pose_arm_default.pose.position.x = 0.1
        sm.userdata.pose_arm_default.pose.position.y = 0.0
        sm.userdata.pose_arm_default.pose.position.z = 0.5
        sm.userdata.pose_arm_default.pose.orientation.w = 1.0
        sm.userdata.result = pick_and_place_msgs.PickObjectResult()
        
        smach.StateMachine.add('ParseGoal',
                               ParseGoal(),
                               remapping={'goal':'goal',
                                          'feedback':'feedback',
                                          'object_pose':'object_pose'},
                               transitions={'parsed':'PickObject'})
        
        sm_pick = pick_object_sm.createSM()
        smach.StateMachine.add('PickObject',
                               sm_pick,
                               remapping={'object_name':'object_name',
                                          'object_pose':'object_pose',
                                          'pose_arm_default':'pose_arm_default'},
                               transitions={'picked':'picked',
                                            'pick_failed':'pick_failed',
                                            'preempted':'preempted',
                                            'error':'error'})
    
    asw = ActionServerWrapper('pick_object',
                              pick_and_place_msgs.PickObjectAction,
                              wrapped_container = sm,
                              goal_key = 'goal',
                              feedback_key = 'feedback',
                              result_key = 'result',
                              succeeded_outcomes = ['picked'],
                              aborted_outcomes = ['pick_failed','error'],
                              preempted_outcomes = ['preempted'])
    
    asw.run_server()
    
    rospy.spin()

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
