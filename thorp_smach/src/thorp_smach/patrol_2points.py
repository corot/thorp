#!/usr/bin/env python

import rospy
import smach
import smach_ros

import geometry_msgs.msg as geometry_msgs
import move_base_msgs.msg as move_base_msgs

from actionlib import *


def main():
    rospy.init_node('smach_example_actionlib')

    sm = smach.StateMachine(outcomes=['success',
                                      'aborted',
                                      'preempted'])
    with sm:
        # general
        sm.userdata.true = True
        sm.userdata.false = False
        # table poses
        sm.userdata.pose_table_a = geometry_msgs.PoseStamped()
        sm.userdata.pose_table_a.header.stamp = rospy.Time.now()
        sm.userdata.pose_table_a.header.frame_id = "map"
        sm.userdata.pose_table_a.pose.position.x = 2.0
        sm.userdata.pose_table_a.pose.position.y = 5.0
        sm.userdata.pose_table_a.pose.orientation.x = 0.0
        sm.userdata.pose_table_a.pose.orientation.y = 0.0
        sm.userdata.pose_table_a.pose.orientation.z = 0.851
        sm.userdata.pose_table_a.pose.orientation.w = 0.526
        sm.userdata.pose_table_b = geometry_msgs.PoseStamped()
        sm.userdata.pose_table_b.header = sm.userdata.pose_table_a.header
        sm.userdata.pose_table_b.pose.position.x = 2.0
        sm.userdata.pose_table_b.pose.position.y = 1.0
        sm.userdata.pose_table_b.pose.orientation.x = 0.0
        sm.userdata.pose_table_b.pose.orientation.y = 0.0
        sm.userdata.pose_table_b.pose.orientation.z = -0.509
        sm.userdata.pose_table_b.pose.orientation.w = 0.861
        # Thorp base pose
        sm.userdata.base_position = geometry_msgs.PoseStamped()

        smach.StateMachine.add('MoveToTableA',
                               smach_ros.SimpleActionState('move_base',
                                                           move_base_msgs.MoveBaseAction,
                                                           goal_slots=['target_pose'],
                                                           result_slots=[]),
                               remapping={'target_pose': 'pose_table_a',
                                          'base_position': 'base_position'},
                               transitions={'succeeded': 'MoveToTableB',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('MoveToTableB',
                               smach_ros.SimpleActionState('move_base',
                                                           move_base_msgs.MoveBaseAction,
                                                           goal_slots=['target_pose'],
                                                           result_slots=[]),
                               remapping={'target_pose': 'pose_table_b',
                                          'base_position': 'base_position'},
                               transitions={'succeeded': 'MoveToTableA',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})

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
