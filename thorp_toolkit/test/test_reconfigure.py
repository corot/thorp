#!/usr/bin/env python

import rospy
import pytest

from thorp_toolkit.reconfigure import Reconfigure


@pytest.fixture
def node():
    rospy.init_node('test_reconfigure', anonymous=True)


def test_update_config(node):
    orig_pf = rospy.get_param('move_base_flex/planner_frequency')
    orig_pp = rospy.get_param('move_base_flex/planner_patience')
    assert Reconfigure().update_config('move_base_flex', {'planner_frequency': 69,
                                                          'planner_patience': 11})
    assert rospy.get_param('move_base_flex/planner_frequency') == 69
    assert rospy.get_param('move_base_flex/planner_patience') == 11
    assert Reconfigure().restore_config('move_base_flex', ['planner_frequency',
                                                           'planner_patience'])
    assert rospy.get_param('move_base_flex/planner_frequency') == orig_pf
    assert rospy.get_param('move_base_flex/planner_patience') == orig_pp


def test_named_configs(node):
    rospy.loginfo("Load named configs")
    Reconfigure().load_named_configs()  # load named configurations from the default location

    rospy.loginfo("Use named configs")
    for config in Reconfigure().named_configs:
        assert Reconfigure().use_named_config(config), "Use named config '%s' failed" % config

    rospy.loginfo("Dismiss named configs")
    assert Reconfigure().dismiss_named_config(config), "Dismiss named config '%s' failed" % config

    assert not Reconfigure().use_named_config('foo'), "Use named config 'foo' worked???"
    assert not Reconfigure().dismiss_named_config('bar'), "Dismiss config 'bar' worked???"
