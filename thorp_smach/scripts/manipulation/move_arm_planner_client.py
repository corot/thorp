#!/usr/bin/env python
import sys
import argparse
from thorp_smach.state_machines.state_machines_imports import *
from thorp_smach.pick_and_place_tools.msg_imports import *


def main():
    rospy.init_node('move_arm_planner_client')
    
    parser = argparse.ArgumentParser(description='Client for Korus` move arm IK action server')
    parser.add_argument('pos_x', type = float, help='X value of the target position')
    parser.add_argument('pos_y', type = float, help='Y value of the target position')
    parser.add_argument('pos_z', type = float, help='Z value of the target position')
    args=parser.parse_args()
    
    goal_pose = geometry_msgs.PoseStamped()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = "base_footprint"
    
    goal_pose.pose.position.x = args.pos_x
    goal_pose.pose.position.y = args.pos_y
    goal_pose.pose.position.z = args.pos_z
    goal_pose.pose.orientation.x = 0.707
    goal_pose.pose.orientation.y = -0.000
    goal_pose.pose.orientation.z = 0.000
    goal_pose.pose.orientation.w = 0.707
    
    rospy.loginfo("Will try to find an IK solution for pose:")
    rospy.loginfo("(" + str(goal_pose.pose.position.x)
                  + ", " + str(goal_pose.pose.position.y)
                  + ", " + str(goal_pose.pose.position.z)
                  + ")(" + str(goal_pose.pose.orientation.x)
                  + ", " + str(goal_pose.pose.orientation.y)
                  + ", " + str(goal_pose.pose.orientation.z)
                  + ", " + str(goal_pose.pose.orientation.w) + ")")
    
    sm = StateMachine(outcomes=['succeeded',
                                'aborted',
                                'preempted'])
    sm.userdata.goal_pose = goal_pose

    with sm:
        def MoveArmResultCb(userdata, status, result):
            rospy.loginfo('action finished in state ' + str(status))
            rospy.loginfo('action result: ')
            rospy.loginfo(str(result))
            
        smach.StateMachine.add('MoveArm',
                               SimpleActionState('/korus/move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose'],
                                                 result_cb=MoveArmResultCb))
    sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass