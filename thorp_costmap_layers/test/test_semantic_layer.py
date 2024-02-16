#!/usr/bin/env python

import numpy as np
from math import pi

import rospy
import pytest

from geometry_msgs.msg import Vector3

from thorp_toolkit.geometry import create_2d_pose
from thorp_toolkit.visualization import Visualization
from thorp_costmap_layers.srv_iface_client import SemanticLayer


@pytest.fixture
def node():
    rospy.init_node('test_semantic_layer')


def test_semantic_layer(node):
    p00 = create_2d_pose(0, 0, 1, 'map')
    p11 = create_2d_pose(1, 2, 1, 'map')
    p22 = create_2d_pose(2, 2, 1, 'map')
    p33 = create_2d_pose(3, 3, 1, 'map')
    p16 = create_2d_pose(1.6, 1.6, 0, 'map')
    sz1 = Vector3(2.0, 2.0, 0.0)
    sz2 = Vector3(1.2, 0.8, 0.0)
    sz3 = Vector3(0.5, 0.5, 0.0)
    sz4 = Vector3(0.75, 0.75, 0.0)

    sl = SemanticLayer()

    # add some obstacles
    assert sl.add_object('obs0', 'blocked_area', p11, sz1, costmap='both')
    assert sl.add_object('fs1', 'free_space', p11, sz1, costmap='local')
    assert sl.remove_object('fs1', 'free_space', costmap='local')
    assert sl.remove_object('obs0', 'blocked_area', costmap='both')
    assert sl.add_object('obs1', 'obstacle', p11, sz1, costmap='both')
    assert sl.add_object('obs2', 'obstacle', p11, sz2, costmap='both')
    assert sl.add_object('obs3', 'obstacle', p11, sz3, costmap='both')
    assert sl.add_object('obs4', 'obstacle', p11, sz4, costmap='both')

    # add some free space
    assert sl.add_object('fs1', 'free_space', p11, sz1, costmap='local')
    assert sl.add_object('fs2', 'free_space', p11, sz2, costmap='local')

    # move the first one
    assert sl.add_object('obs1', 'obstacle', p33, sz1, costmap='local')

    # resize the second one
    assert sl.add_object('obs2', 'obstacle', p11, sz1, costmap='local')

    # remove un-existing object
    assert not sl.remove_object('xxx', 'obstacle', costmap='local')

    # remove all objects
    assert sl.remove_object('fs2', 'free_space', costmap='local')
    assert sl.remove_object('fs1', 'free_space', costmap='local')
    assert sl.remove_object('obs4', 'obstacle', costmap='local')
    assert sl.remove_object('obs3', 'obstacle', costmap='local')
    assert sl.remove_object('obs2', 'obstacle', costmap='local')
    assert sl.remove_object('obs1', 'obstacle', costmap='local')

    # Visualization().add_box_marker(p11, sz1 + (0.0001,), Visualization.rand_color(0.2))
    # Visualization().publish_markers()
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


def test_3_boxes_challenge(node):
    p00 = create_2d_pose(4.0, 0.5, 0.0, 'map')
    p11 = create_2d_pose(4.8, 1.0, 0.0, 'map')
    p22 = create_2d_pose(5.6, 0.5, 0.0, 'map')
    sz3 = (0.35, 0.35)

    sl = SemanticLayer()

    # add 3 obstacles forming a difficult narrow passage
    assert sl.add_object('obs1', 'obstacle', p00, sz3, costmap='both')
    assert sl.add_object('obs2', 'obstacle', p11, sz3, costmap='both')
    assert sl.add_object('obs3', 'obstacle', p22, sz3, costmap='both')
