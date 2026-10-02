#!/usr/bin/env python3

"""
Show the currently running BT node on RViz in the top-left corner
Author:
    Jorge Santos
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rviz_2d_overlay_msgs.msg import OverlayText
from thorp_msgs.msg import BTNodeStatus
from thorp_toolkit.visualization import Visualization

# nodes of secondary relevance that run in parallel with more critical ones
BLACKLISTED_NODES = ['Sleep', 'TrackProgress']


class ShowBTNodeOnRViz(Node):

    def __init__(self):
        super().__init__('show_bt_node_on_rviz')
        app_name = self.declare_parameter('app_name', '').value
        self.status_pub = self.create_publisher(OverlayText, 'rviz/executive_progress_overlay', 1)
        # bt_runner's node is named after the app
        self.create_subscription(BTNodeStatus, f'/{app_name}/bt_status', self.bt_status_cb, 5)

    def bt_status_cb(self, msg):
        if msg.name in BLACKLISTED_NODES:
            return

        # Publish only subtrees and actions the first tick they start running
        if msg.type not in [BTNodeStatus.ACTION, BTNodeStatus.SUBTREE]:
            return

        if msg.prev_status == BTNodeStatus.IDLE and msg.status == BTNodeStatus.RUNNING:
            self.status_pub.publish(Visualization.create_overlay_text(60, (1.0, 1.0, 1.0), msg.name, 12))


def main():
    rclpy.init()
    node = ShowBTNodeOnRViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
