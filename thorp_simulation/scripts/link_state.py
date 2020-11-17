#!/usr/bin/env python

"""
Print pose and velocity in Gazebo for a given link
"""

import sys
import rospy

from gazebo_msgs.msg import LinkStates

from thorp_toolkit.geometry import pose3d2str


def link_states_cb(msg):
    try:
        index = msg.name.index('thorp::' + sys.argv[1])
        print(pose3d2str(msg.pose[index]))
        print(msg.twist[index])
    except ValueError:
        pass


if __name__ == "__main__":
    rospy.init_node("link_state")
    rospy.Subscriber("gazebo/link_states", LinkStates, link_states_cb)
    rospy.spin()
