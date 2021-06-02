#!/usr/bin/env python

import rospy
import actionlib

from random import choice

from thorp_toolkit.geometry import translate_pose

from geometry_msgs.msg import PoseStamped
from thorp_msgs.msg import FollowPoseAction, FollowPoseGoal, FollowPoseResult

__author__ = 'Jorge Santos'

""" 
Call pose follower to reach a moving target pose provided by 2D nav goal RViz tool.
"""


def nav_goal_cb(msg):
    global target_pose
    target_pose = msg
    goal = FollowPoseGoal(stop_at_distance=True, time_limit=rospy.Time(10))
    follow_ac.send_goal(goal, done_cb=follow_done_cb)
    axis = choice(['x', 'y'])
    delta = choice([-0.1, 0.1])
    while not follow_ac.wait_for_result(rospy.Duration(0.5)) and not rospy.is_shutdown():
        msg.header.stamp = rospy.get_rostime()
        translate_pose(msg, delta, axis)
        target_pub.publish(msg)


def follow_done_cb(status, result):
    if result.outcome == FollowPoseResult.WITHIN_DISTANCE:
        rospy.loginfo("Follow pose arrived within distance")
    else:
        rospy.logerr("Follow pose finished with %d", result.outcome)


if __name__ == '__main__':
    rospy.init_node("call_pose_follower")

    follow_ac = actionlib.SimpleActionClient('pose_follower/follow', FollowPoseAction)
    follow_ac.wait_for_server(rospy.Duration(5))

    target_pose = None
    target_pub = rospy.Publisher('/target_object_pose', PoseStamped, queue_size=1)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, nav_goal_cb)

    rospy.spin()
