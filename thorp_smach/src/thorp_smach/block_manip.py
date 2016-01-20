#!/usr/bin/env python

import rospy
import smach
import smach_ros

import geometry_msgs.msg as geometry_msgs
import move_base_msgs.msg as move_base_msgs

from actionlib import *
from actionlib_msgs.msg import *
from turtlebot_arm_block_manipulation.msg import *


def monitor_cb(ud, msg):
    return True

def main():
    rospy.init_node('smach_block_manip')

    sm = smach.StateMachine(outcomes=['success',
                                      'aborted',
                                      'preempted'])
    with sm:
        ''' general '''
        sm.userdata.true = True
        sm.userdata.false = False
        ''' table poses '''
        sm.userdata.frame          = rospy.get_param('~arm_link', '/arm_link')
        sm.userdata.gripper_open   = rospy.get_param('~gripper_open', 0.042)
        sm.userdata.gripper_closed = rospy.get_param('~gripper_closed', 0.024)
        sm.userdata.z_up           = rospy.get_param('~z_up', 0.12)
        sm.userdata.table_height   = rospy.get_param('~table_height', 0.1)
        sm.userdata.block_size     = rospy.get_param('~block_size', 0.025)
        sm.userdata.blocks         = geometry_msgs.msg.PoseArray()
        sm.userdata.pickup_pose    = geometry_msgs.msg.Pose()
        sm.userdata.place_pose     = geometry_msgs.msg.Pose()
        sm.userdata.topic          = ''

        smach.StateMachine.add('BlockDetection',
                               smach_ros.SimpleActionState('block_detection',
                                                           BlockDetectionAction,
                                                           goal_slots=['frame', 'table_height', 'block_size'],
                                                           result_slots=['blocks']),
                               remapping={'frame':'frame',
                                          'table_height':'table_height',
                                          'block_size':'block_size',
                                          'blocks':'blocks'},
                               transitions={'succeeded':'InteractiveManip',
                                            'aborted':'aborted',
                                            'preempted':'preempted'})

        smach.StateMachine.add('InteractiveManip',
                               smach_ros.SimpleActionState('interactive_manipulation',
                                                           InteractiveBlockManipulationAction,
                                                           goal_slots=['frame', 'block_size'],
                                                           result_slots=['pickup_pose', 'place_pose']),
                               remapping={'frame':'frame',
                                          'block_size':'block_size',
                                          'pickup_pose':'pickup_pose',
                                          'place_pose':'place_pose'},
                               transitions={'succeeded':'PickAndPlace',
                                            'aborted':'BlockDetection',
                                            'preempted':'preempted'})

        smach.StateMachine.add('PickAndPlace',
                               smach_ros.SimpleActionState('pick_and_place',
                                                           PickAndPlaceAction,
                                                           goal_slots=['frame', 'z_up',
                                                                       'gripper_open', 'gripper_closed',
                                                                       'pickup_pose', 'place_pose', 'topic'],
                                                           result_slots=[]),
                               remapping={'frame':'frame',
                                          'z_up':'z_up',
                                          'gripper_open':'gripper_open',
                                          'gripper_closed':'gripper_closed',
                                          'pickup_pose':'pickup_pose',
                                          'place_pose':'place_pose'},
                               transitions={'succeeded':'BlockDetection',
                                            'aborted':'BlockDetection',
                                            'preempted':'preempted'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()
