#!/usr/bin/env python

import sys
import argparse

# ros basics& smach
import roslib; roslib.load_manifest('thorp_smach')
import rospy
import smach
from smach import StateMachine
import smach_ros
from smach_ros import SimpleActionState

from thorp_smach.msg import MoveGripperAction
#from thorp_smach.msg import MoveArmIKGoal
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('move_gripper_client')
    
    parser = argparse.ArgumentParser(description='Client for Korus` move gripper action server')
    parser.add_argument('gripper_angle', type = float, help='gripper angle')
    args=parser.parse_args()
    
    sm = StateMachine(outcomes=['succeeded',
                                'aborted',
                                'preempted'])
    sm.userdata.gripper_angle = args.gripper_angle

    with sm:
        def MoveGripperResultCb(userdata, status, result):
            rospy.loginfo('action finished in state ' + str(status))
            rospy.loginfo('action result: ')
            rospy.loginfo(str(result))
            
        smach.StateMachine.add('MoveGripper',
                               SimpleActionState('/korus/move_gripper',
                                                 MoveGripperAction,
                                                 goal_slots=['gripper_angle'],
                                                 result_cb=MoveGripperResultCb))
    sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass