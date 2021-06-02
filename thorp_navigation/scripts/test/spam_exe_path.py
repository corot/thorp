#!/usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from mbf_msgs.msg import GetPathAction, GetPathGoal, GetPathResult
from mbf_msgs.msg import ExePathAction, ExePathGoal, ExePathResult

__author__ = 'Jorge Santos'

""" 
Continuously call exe_path action.
"""

exe_path_goal = None


def nav_goal_cb(msg):
    global target_pose
    target_pose = msg
    goal = GetPathGoal(use_start_pose=False,
                       start_pose=msg,
                       target_pose=msg)
    get_path_ac.send_goal(goal, done_cb=plan_done_cb)


def plan_done_cb(status, result):
    if result.outcome == GetPathResult.SUCCESS:
        rospy.loginfo("Get path action succeeded")

        global exe_path_goal
        exe_path_goal = ExePathGoal(path=result.path, controller="PoseFollower")
        exe_path_ac.send_goal(exe_path_goal, done_cb=follow_done_cb)
    else:
        rospy.logerr("Get path action failed with error code [%d]: %s", result.outcome, result.message)


def follow_done_cb(status, result):
    if result.outcome == GetPathResult.SUCCESS:
        rospy.loginfo("Follow path action succeeded")
    else:
        rospy.logerr("Follow path action failed with error code [%d]: %s", result.outcome, result.message)
    #rospy.sleep(0.5)  TODO uncomment to make current code work
    exe_path_ac.send_goal(exe_path_goal, done_cb=follow_done_cb)


if __name__ == '__main__':
    rospy.init_node("spam_exe_path")

    target_pose = None

    get_path_ac = actionlib.SimpleActionClient("/move_base_flex/get_path", GetPathAction)
    get_path_ac.wait_for_server(rospy.Duration(5))
    exe_path_ac = actionlib.SimpleActionClient("/move_base_flex/exe_path", ExePathAction)
    exe_path_ac.wait_for_server(rospy.Duration(5))

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, nav_goal_cb)

    rospy.spin()
