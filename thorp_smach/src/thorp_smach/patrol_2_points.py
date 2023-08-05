#!/usr/bin/env python

import rospy
import smach

import geometry_msgs.msg as geometry_msgs

from thorp_smach.states.navigation import GoToPose

from thorp_smach.utils import run_sm


def patrol_2_points_sm():
    sm = smach.StateMachine(outcomes=['success', 'aborted', 'preempted'])
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

        smach.StateMachine.add('MoveToTableA', GoToPose(),
                               transitions={'succeeded': 'MoveToTableB',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'},
                               remapping={'target_pose': 'pose_table_a'})

        smach.StateMachine.add('MoveToTableB', GoToPose(),
                               transitions={'succeeded': 'MoveToTableA',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'},
                               remapping={'target_pose': 'pose_table_b'})

    return sm


if __name__ == '__main__':
    rospy.init_node('patrol_2_points_smach')

    target_types = rospy.get_param('~object_types', '').split()
    run_sm(patrol_2_points_sm(), rospy.get_param('~app_name'))
