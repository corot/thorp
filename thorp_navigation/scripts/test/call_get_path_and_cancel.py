#!/usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from thorp_toolkit.geometry import pose2d2str

from mbf_msgs.msg import GetPathAction, GetPathGoal, GetPathResult
from mbf_msgs.msg import ExePathAction, ExePathGoal, ExePathResult

__author__ = 'Jorge Santos'

""" 
Call exe_path action and cancel immediately.
"""


def nav_goal_cb(msg):
    global target_pose
    target_pose = msg
    goal = GetPathGoal(use_start_pose=False,
                       start_pose=msg,
                       target_pose=msg,
                       planner="planner2")
    get_path_ac.send_goal(goal, done_cb=plan_done_cb)
    rospy.loginfo("Relaying move_base_simple/goal %s from RViz to MBF's planner", pose2d2str(msg))
    rospy.sleep(5)
    get_path_ac.cancel_goal()


def plan_done_cb(status, result):
    if result.outcome == GetPathResult.SUCCESS:
        rospy.loginfo("Get path action succeeded")
        # result.path.poses.insert(0, result.path.poses[0])   create a fake RIP segment
        # print result.path
        get_path_ac.cancel_all_goals()
        # global target_pose
        # if target_pose:
        #     goal = GetPathGoal(use_start_pose=False,
        #                        start_pose=target_pose,
        #                        target_pose=target_pose,
        #                    planner="planner2")
        #     follow_ac.send_goal(goal, done_cb=plan_done_cb)
        #     target_pose = None
    else:
        rospy.logerr("Get path action failed with error code [%d]: %s", result.outcome, result.message)


def follow_done_cb(status, result):
    if result.outcome == GetPathResult.SUCCESS:
        rospy.loginfo("Follow path action succeeded")
    else:
        rospy.logerr("Follow path action failed with error code [%d]: %s", result.outcome, result.message)


if __name__ == '__main__':
    rospy.init_node("call_get_path_and_cancel")

    target_pose = None

    get_path_ac = actionlib.SimpleActionClient("/move_base_flex/get_path", GetPathAction)
    get_path_ac.wait_for_server(rospy.Duration(5))
    exe_path_ac = actionlib.SimpleActionClient("/move_base_flex/exe_path", ExePathAction)
    exe_path_ac.wait_for_server(rospy.Duration(5))

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, nav_goal_cb)

    rospy.spin()
