#!/usr/bin/env python

import rospy
import actionlib

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from thorp_msgs.msg import FollowPoseAction
from turtlebot_msgs.srv import SetFollowState

__author__ = 'Jorge Santos'

""" 
Call TEB with a single-pose path.
"""

exe_path_goal = None


def nav_goal_cb(msg):
    global target_pose
    target_pose = msg
    goal = ExePathGoal(controller='TEBPlanner', # 'PoseFollower',
                       #angle_tolerance=1000,
                       path=Path(msg.header, [msg]))
    exe_path_ac.send_goal(goal, done_cb=follow_done_cb)


def follow_done_cb(status, result):
    if result.outcome == GetPathResult.SUCCESS:
        rospy.loginfo("Follow path action succeeded")
    else:
        rospy.logerr("Follow path action failed with error code [%d]: %s", result.outcome, result.message)
    #rospy.sleep(0.5)  TODO uncomment to make current code work
    #exe_path_ac.send_goal(exe_path_goal, done_cb=follow_done_cb)


if __name__ == '__main__':
    rospy.init_node("call_pose_follower")

    target_pose = None

    follow_ac = actionlib.SimpleActionClient('pose_follower/follow', FollowPoseAction)
    follow_ac.wait_for_server(rospy.Duration(5))
    exe_path_ac = actionlib.SimpleActionClient("/move_base_flex/exe_path", ExePathAction)
    exe_path_ac.wait_for_server(rospy.Duration(5))

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, nav_goal_cb)

    rospy.spin()
