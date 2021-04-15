#!/usr/bin/env python

import rospy
import pytest

from thorp_toolkit.geometry import create_2d_pose
from thorp_toolkit.visualization import Visualization
from thorp_toolkit.semantic_map import SemanticMap


@pytest.fixture
def node():
    rospy.init_node('test_semantic_map')


def test_semantic_map(node):
    p00 = create_2d_pose(0, 0, 0, 'map')
    p11 = create_2d_pose(1, 1, 0, 'map')
    p22 = create_2d_pose(2, 2, 0, 'map')
    p33 = create_2d_pose(3, 3, 0, 'map')
    p16 = create_2d_pose(1.6, 1.6, 0, 'map')

    sm = SemanticMap()

    # add_object
    sm.add_object('obj1', 'dummy', p11, (1, 1))

    Visualization().add_box_marker(p00, (1, 1, 0.0001), Visualization.rand_color(0.2))
    Visualization().publish_markers()
    assert len(sm.objects_at(p00, (1, 1))) == 1
    assert len(sm.objects_at(p00, (0.9, 0.9))) == 0

    Visualization().add_box_marker(p22, (1, 1, 0.0001), Visualization.rand_color(0.2))
    Visualization().publish_markers()
    assert len(sm.objects_at(p22, (1, 1))) == 1

    Visualization().add_box_marker(p33, (1, 1, 0.0001), Visualization.rand_color(0.2))
    Visualization().publish_markers()
    assert len(sm.objects_at(p33, (1, 1))) == 0

    sm.add_object('obj2', 'dummy', p22, (0.5, 0.5))

    Visualization().add_box_marker(p16, (1, 1, 0.0001), Visualization.rand_color(0.2))
    Visualization().publish_markers()
    assert len(sm.objects_at(p16, (1, 1))) == 2

    # remove_object
    sm.remove_object('obj1')
    assert len(sm.objects_at(p16, (1, 1))) == 1

    sm.remove_object('obj2')
    assert len(sm.objects_at(p16, (1, 1))) == 0

    # objects count
    sm.add_object('obj1', 'dummy', p11, (1, 1))
    sm.add_object('obj2', 'dummy', p22, (0.5, 0.5))
    sm.add_object('obj3', 'dummy', p33, (0.5, 0.5))
    assert sm.objects_count() == 3
    assert sm.objects() == {'obj1', 'obj2', 'obj3'}
    sm.remove_object('obj1')
    sm.remove_object('obj2')
    sm.remove_object('obj3')
    assert sm.objects_count() == 0
    assert sm.objects() == set()
