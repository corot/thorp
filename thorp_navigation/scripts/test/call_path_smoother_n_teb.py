#!/usr/bin/env python
import math

import rospy
import actionlib

from tf.transformations import quaternion_from_euler

from nav_msgs.msg import Path
from mbf_msgs.msg import ExePathAction, ExePathGoal, ExePathResult

from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from thorp_msgs.srv import ConnectWaypoints

""" Just run goal busy task for debugging isolated """


def move_goal_cb(_):
    global waypoints
    try:
        resp = smooth_srv(waypoints, 5, 0.5, 0.15, True, True)
        path_pub.publish(resp.path)
        print resp.path
        exe_path_goal = ExePathGoal(path=resp.path, controller="TEBPlanner")
        exe_path_ac.send_goal(exe_path_goal)
    except rospy.ServiceException as err:
        rospy.logerr("Call connect_waypoints failed: %s", str(err))
    waypoints = []


def point_cb(msg):
    print(msg.point)
    global waypoints
    pose = PoseStamped()
    pose.header = msg.header
    pose.pose.position.x = msg.point.x
    pose.pose.position.y = msg.point.y
    if not waypoints:
        yaw = 0
    else:
        yaw = math.atan2(pose.pose.position.y - waypoints[-1].pose.position.y,
                         pose.pose.position.x - waypoints[-1].pose.position.x)
        # Set yaw of previous pose to point towards next segment on the path
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = \
        quaternion_from_euler(0, 0, yaw)
    waypoints.append(pose)
    viz_waypoints()


def viz_waypoints():
    markers = MarkerArray()

    path_marker = Marker()
    path_marker.header = waypoints[0].header
    path_marker.ns = 'path'
    path_marker.id = len(markers.markers)
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = path_marker.ADD
    path_marker.points = [p.pose.position for p in waypoints]
    path_marker.color.b = 1.
    path_marker.color.a = 1.
    path_marker.scale.x = .05
    markers.markers.append(path_marker)

    wp_marker = Marker()
    wp_marker.header = waypoints[0].header
    wp_marker.ns = 'waypoints'
    wp_marker.id = len(markers.markers)
    wp_marker.type = Marker.SPHERE_LIST
    wp_marker.action = Marker.ADD
    wp_marker.points = [p.pose.position for p in waypoints]
    wp_marker.color.r = 1.
    wp_marker.color.a = 1.
    wp_marker.scale.x = .25
    wp_marker.scale.y = .25
    wp_marker.scale.z = .25
    markers.markers.append(wp_marker)

    dir_marker_del = Marker()
    dir_marker_del.ns = 'direction'
    dir_marker_del.action = Marker.DELETEALL
    markers.markers.append(dir_marker_del)
    for wp in waypoints:
        dir_marker = Marker()
        dir_marker.ns = 'direction'
        dir_marker.header = wp.header
        dir_marker.action = Marker.ARROW
        dir_marker.action = Marker.ADD
        dir_marker.ns = 'direction'
        dir_marker.id = len(markers.markers)
        dir_marker.pose = wp.pose
        dir_marker.color.r = 1.
        dir_marker.color.g = 1.
        dir_marker.color.b = 0.
        dir_marker.color.a = 0.5
        dir_marker.scale.x = .5
        dir_marker.scale.y = .2
        dir_marker.scale.z = .05
        markers.markers.append(dir_marker)

    viz_pub.publish(markers)


if __name__ == '__main__':
    rospy.init_node("call_path_smoother")

    waypoints = []
    rospy.Subscriber('move_base_simple/goal', PoseStamped, move_goal_cb)
    rospy.Subscriber('clicked_point', PointStamped, point_cb)
    path_pub = rospy.Publisher('~output_path', Path, queue_size=1)
    viz_pub = rospy.Publisher('~visualization', MarkerArray, queue_size=1)
    smooth_srv = rospy.ServiceProxy('waypoints_path/connect_waypoints', ConnectWaypoints)
    exe_path_ac = actionlib.SimpleActionClient("move_base_flex/exe_path", ExePathAction)
    exe_path_ac.wait_for_server(rospy.Duration(5))
    rospy.spin()
