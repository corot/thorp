#!/usr/bin/env python

import time

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from thorp_toolkit.geometry import pose2d2str

from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

__author__ = 'Jorge Santos'

""" 
Call move_base action and cancel immediately.
"""


def nav_goal_cb(msg):
    rospy.loginfo("Calling MBF's move_base action with target pose %s", pose2d2str(msg))
    goal = MoveBaseGoal(target_pose=msg)
    move_base_ac.send_goal(goal, done_cb=move_done_cb)
  #  rospy.sleep(0.000005)#random.randrange(300, 800)/100.0)        hostias!!! tarda bastante, aunque pongas epsilon!!! (en sim, supongo)
    time.sleep(2.5)
    move_base_ac.cancel_goal()
    rospy.loginfo("CANCEL!!!")


def move_done_cb(status, result):
    if result.outcome == MoveBaseResult.SUCCESS:
        rospy.loginfo("Follow path action succeeded")
    else:
        rospy.logerr("Follow path action failed with error code [%d]: %s", result.outcome, result.message)


if __name__ == '__main__':
    rospy.init_node("call_move_base_and_cancel")

    move_base_ac = actionlib.SimpleActionClient("/move_base_flex/move_base", MoveBaseAction)
    move_base_ac.wait_for_server(rospy.Duration(5))

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, nav_goal_cb)

    rospy.spin()
