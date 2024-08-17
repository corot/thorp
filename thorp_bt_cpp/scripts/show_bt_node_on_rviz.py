#!/usr/bin/env python

"""
Show current BT node on RViz at top-left corner
Author:
    Jorge Santos
"""

import re

import rospy

from std_msgs.msg import String

from jsk_rviz_plugins.msg import OverlayText

from thorp_toolkit.visualization import Visualization


# nodes of secondary relevance run in parallel with more critical one
BLACKLISTED_NODES = ['TrackProgress']


def bt_status_cb(msg):
    if msg.data in BLACKLISTED_NODES:
        return

    overlay_text = Visualization.create_overlay_text(60, (1.0, 1.0, 1.0), msg.data, 12)
    status_pub.publish(overlay_text)


if __name__ == "__main__":
    rospy.init_node("show_bt_node_on_rviz")

    status_pub = rospy.Publisher('rviz/executive_progress_overlay', OverlayText, queue_size=1)

    status_topic = rospy.get_param('~app_name') + '/bt_status'
    cs_sub = rospy.Subscriber(status_topic, String, bt_status_cb, queue_size=5)

    rospy.spin()
