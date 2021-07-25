#!/usr/bin/env python

import numpy as np
from math import pi

import rospy
import pytest

from thorp_toolkit.geometry import create_2d_pose
from thorp_toolkit.visualization import Visualization
from thorp_toolkit.semantic_layer import SemanticLayer


@pytest.fixture
def node():
    rospy.init_node('test_semantic_layer')


def test_semantic_layer(node):
    p00 = create_2d_pose(0, 0, 0, 'map')
    p11 = create_2d_pose(2, 2, 0, 'map')
    p22 = create_2d_pose(2, 2, 0, 'map')
    p33 = create_2d_pose(3, 3, 0, 'map')
    p16 = create_2d_pose(1.6, 1.6, 0, 'map')
    sz1 = (3.0, 3.0)
    sz2 = (1.2, 0.8)
    sz3 = (0.5, 0.5)
    sz4 = (0.75, 0.75)

    sl = SemanticLayer()
    sl.add_object('obs1', 'obstacle', p11, sz1, costmap='local')
#    rospy.spin()
    sl.add_object('obs2', 'obstacle', p11, sz2, costmap='local')
    sl.add_object('obs3', 'obstacle', p11, sz3, costmap='local')
    sl.add_object('obs4', 'obstacle', p11, sz4, costmap='local')
    sl.add_object('fs1', 'free_space', p11, sz1, costmap='local')
    sl.add_object('fs2', 'free_space', p11, sz2, costmap='local')
    Visualization().add_box_marker(p11, sz1 + (0.0001,), Visualization.rand_color(0.2))
    Visualization().publish_markers()
    # sl.add_object('obj2', 'obstacle', p33, sz1, costmap='local')
    # Visualization().add_box_marker(p11, sz1 + (0.0001,), Visualization.rand_color(0.2))
    # Visualization().publish_markers()
    sl.remove_object('obj3', 'obstacle', costmap='local')
    sl.remove_object('obj2', 'obstacle', costmap='local')
    sl.remove_object('obj1', 'obstacle', costmap='local')
    pass
    # assert len(sl.objects_at(p00, (1, 1))) == 1
    # print sl.objects_at(p00, (1, 1))
    # assert len(sl.objects_at(p00, (0.9, 0.9))) == 0
    # print sl.objects_at(p00, (0.9, 0.9))
    # Visualization().add_box_marker(p00, (1, 1, 0.0001), Visualization.rand_color(0.2))
    # Visualization().publish_markers()
    # assert len(sl.objects_at(p22, (1, 1))) == 1
    # Visualization().add_box_marker(p22, (1, 1, 0.0001), Visualization.rand_color(0.2))
    # Visualization().publish_markers()
    # assert len(sl.objects_at(p33, (1, 1))) == 0
    # Visualization().add_box_marker(p33, (1, 1, 0.0001), Visualization.rand_color(0.2))
    # Visualization().publish_markers()
    # sl.add_object('obj2', 'dummy', create_2d_pose(2, 2, 0, 'map'), (0.5, 0.5))
    # Visualization().add_box_marker(p22, (0.5, 0.5, 0.0001), Visualization.rand_color(0.2))
    # Visualization().publish_markers()
    # assert len(sl.objects_at(p16, (1, 1))) == 2
    # Visualization().add_box_marker(p16, (1, 1, 0.0001), Visualization.rand_color(0.2))
    # Visualization().publish_markers()
