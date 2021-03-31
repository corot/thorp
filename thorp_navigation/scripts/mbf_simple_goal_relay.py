#!/usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from thorp_toolkit.geometry import pose2d2str

from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

__author__ = 'Jorge Santos'

""" 
Relay goals provided through RViz on 2D Nav Goal tool to move_base_flex's move_base action
"""


def nav_goal_cb(msg):
    rospy.loginfo("Calling MBF's move_base action with target pose %s", pose2d2str(msg))
    goal = MoveBaseGoal(target_pose=msg)
    move_base_ac.send_goal(goal, done_cb=nav_done_cb)


def nav_done_cb(status, result):
    if result.outcome == MoveBaseResult.SUCCESS:
        rospy.loginfo("move_base action succeeded")
    else:
        rospy.logerr("move_base action failed with error code [%d]: %s", result.outcome, result.message)


if __name__ == '__main__':
    rospy.init_node("mbf_simple_goal_relay")

    move_base_ac = actionlib.SimpleActionClient("/move_base_flex/move_base", MoveBaseAction)
    move_base_ac.wait_for_server(rospy.Duration(30))

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, nav_goal_cb)

    rospy.spin()
