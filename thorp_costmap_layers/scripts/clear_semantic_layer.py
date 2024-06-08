#!/usr/bin/env python

"""
Remove all objects from the semantic layer on both the local and global costmaps.
"""

import rospy

from geometry_msgs.msg import Point
from thorp_costmap_layers.srv_iface_client import SemanticLayer


def main():
    rospy.init_node('remove_all_objects')

    semantic_layer = SemanticLayer()

    lower_left = Point()
    lower_left.x = -1000  # Define a sufficiently large bounding box
    lower_left.y = -1000
    upper_right = Point()
    upper_right.x = 1000
    upper_right.y = 1000

    # Remove each object from both costmaps
    for obj in semantic_layer.query_objects(lower_left, upper_right, costmap='local'):
        if not semantic_layer.remove_object(obj.name, obj.type, costmap='local'):
            rospy.logerr(f"Failed to remove object {obj.name} of type {obj.type} on local costmap")
    for obj in semantic_layer.query_objects(lower_left, upper_right, costmap='global'):
        if not semantic_layer.remove_object(obj.name, obj.type, costmap='global'):
            rospy.logerr(f"Failed to remove object {obj.name} of type {obj.type} on global costmap")


if __name__ == '__main__':
    main()
