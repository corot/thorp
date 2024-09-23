#!/usr/bin/env python

"""
Show the currently running BT node on RViz in the top-left corner
Author:
    Jorge Santos
"""

import rospy

from jsk_rviz_plugins.msg import OverlayText

from thorp_msgs.msg import BTNodeStatus
from thorp_toolkit.visualization import Visualization


# nodes of secondary relevance that run in parallel with more critical ones
BLACKLISTED_NODES = ['Sleep', 'TrackProgress']


def bt_status_cb(msg):
    if msg.name in BLACKLISTED_NODES:
        return

    # Publish only subtrees and actions the first tick they start running
    if msg.type not in [BTNodeStatus.ACTION, BTNodeStatus.SUBTREE]:
        return

    if msg.prev_status == BTNodeStatus.IDLE and msg.status == BTNodeStatus.RUNNING:
        overlay_text = Visualization.create_overlay_text(60, (1.0, 1.0, 1.0), msg.name, 12)
        status_pub.publish(overlay_text)


if __name__ == "__main__":
    rospy.init_node("show_bt_node_on_rviz")

    status_pub = rospy.Publisher('rviz/executive_progress_overlay', OverlayText, queue_size=1)

    status_topic = rospy.get_param('~app_name') + '/bt_status'
    cs_sub = rospy.Subscriber(status_topic, BTNodeStatus, bt_status_cb, queue_size=5)

    rospy.spin()
