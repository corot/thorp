#!/usr/bin/env python

"""
Show current SMACH state on RViz at top-left corner
Author:
    Jorge Santos
"""

import re

import rospy

from smach_ros import introspection
from smach_msgs.msg import SmachContainerStatus

from jsk_rviz_plugins.msg import OverlayText

from thorp_toolkit.visualization import Visualization


def smach_status_cb(msg):
    # search for re ['<state name>'] on info field
    # (it looks something like "(<smach.user_data.UserData object at 0x7f0889b1e490>, ['PICKUP_OBJECT']), {}")
    match = re.search(r'\[\'([A-Za-z0-9_]+)\'\]', msg.info)
    if match is None:
        return
    current_state = match.group(1)
    overlay_text = Visualization.create_overlay_text(60, (1.0, 1.0, 1.0), current_state, 12)

    state_pub.publish(overlay_text)


if __name__ == "__main__":
    rospy.init_node("show_smach_state_on_rviz")

    state_pub = rospy.Publisher('rviz/smach_state_overlay', OverlayText, queue_size=1)

    server_name = rospy.get_param('~app_name')
    status_topic = server_name + introspection.STATUS_TOPIC
    cs_sub = rospy.Subscriber(status_topic, SmachContainerStatus, smach_status_cb, queue_size=5)

    rospy.spin()
