#! /usr/bin/env python

import math
import rospy

import actionlib

import thorp_msgs.msg as thorp_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs


def pickup(object):
    # Creates the SimpleActionClient, passing the type of the action
    # (PickupObjectAction) to the constructor.
    client = actionlib.SimpleActionClient('/pickup_object', thorp_msgs.PickupObjectAction)

    # Waits until the action server has started up and started listening for goals
    client.wait_for_server()

    goal = thorp_msgs.PickupObjectGoal(object_name=object, support_surf='table')

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_pickup_object')
        result = pickup('tower 5.0cm [1]')
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
