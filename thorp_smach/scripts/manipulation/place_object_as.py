#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('thorp_smach')
import thorp_smach
from thorp_smach.state_machines import place_object_sm
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
                                          'place_pose',
                                          'feedback'])

    def execute(self, userdata):
        rospy.loginfo('Parsing pick object goal ...')
        userdata.object_name = userdata.goal.object_name
        place_pose = geometry_msgs.PoseStamped()
        place_pose = userdata.goal.place_pose
#        object_pose.pose = userdata.goal.object_pose.pose
#        object_pose.pose.position.x = userdata.goal.object_pose.pose.position.x - 0.15
#        object_pose.pose.position.z = userdata.goal.object_pose.pose.position.z + 0.10
        userdata.place_pose = place_pose
        feedback = pick_and_place_msgs.PlaceObjectFeedback()
        feedback.process_state = 'PlaceObjectGoal parsed'
        userdata.feedback = feedback
        rospy.loginfo('Gripper goal prepared.')
        return 'parsed'

def main():
    rospy.init_node('place_object')
    
    sm = smach.StateMachine(outcomes=['placed',
                                      'place_failed',
                                      'preempted',
                                      'error'],
                            input_keys=['goal'],
                            output_keys=['feedback',
                                         'result'])

    with sm:
        
        sm.userdata.result = pick_and_place_msgs.PickObjectResult()
        
        smach.StateMachine.add('ParseGoal',
                               ParseGoal(),
                               remapping={'goal':'goal',
                                          'feedback':'feedback',
                                          'place_pose':'place_pose'},
                               transitions={'parsed':'PlaceObject'})
        
        sm_place = place_object_sm.createSM()
        smach.StateMachine.add('PlaceObject',
                               sm_place,
                               remapping={'object_name':'object_name',
                                          'place_pose':'place_pose'},
                               transitions={'placed':'placed',
                                            'place_failed':'place_failed',
                                            'preempted':'preempted',
                                            'error':'error'})
    
    asw = ActionServerWrapper('place_object',
                              pick_and_place_msgs.PlaceObjectAction,
                              wrapped_container = sm,
                              goal_key = 'goal',
                              feedback_key = 'feedback',
                              result_key = 'result',
                              succeeded_outcomes = ['placed'],
                              aborted_outcomes = ['place_failed','error'],
                              preempted_outcomes = ['preempted'])
    
    asw.run_server()
    
    rospy.spin()

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
