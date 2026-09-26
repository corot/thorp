from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

import rosgraph_msgs.msg as rosgraph_msgs

_node = None


def init(node: Node):
    """
    Initialize the toolkit with the node its singletons and functions use to access ROS.
    Call once, before using anything else that needs a node.
    """
    global _node
    _node = node


def node() -> Node:
    """ The node given to init """
    if _node is None:
        raise RuntimeError("thorp_toolkit not initialized; call thorp_toolkit.common.init(node) first")
    return _node


def wait_for_sim_time():
    """
    In sim, wait for clock to start (I start gazebo paused, so smach action clients start waiting at time 0,
    but first clock marks ~90s, after spawner unpauses physics)
    """
    if node().get_parameter('use_sim_time').value:
        received, _ = wait_for_message(rosgraph_msgs.Clock, node(), '/clock', time_to_wait=60.0)
        if not received:
            node().get_logger().fatal("No clock msgs after 60 seconds, being use_sim_time true")
            return False
    return True
