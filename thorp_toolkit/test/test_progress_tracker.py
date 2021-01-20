#!/usr/bin/env python

import numpy as np

import rospy
import pytest

from thorp_toolkit.geometry import create_2d_pose
from thorp_toolkit.visualization import Visualization
from thorp_toolkit.progress_tracker import ProgressTracker


@pytest.fixture
def node():
    rospy.init_node('test_progress_tracker')


def test_progress_tracker(node):
    wp = [create_2d_pose(-1, -1, 0, 'map'),
          create_2d_pose(+1, -1, 0, 'map'),
          create_2d_pose(+1, +1, 0, 'map'),
          create_2d_pose(-1, +1, 0, 'map')]
    origin = create_2d_pose(0, 0, 0, 'map')
    Visualization().add_point_markers(wp, 0.1)
    pt = ProgressTracker(wp, np.sqrt(2) - 0.99)
    for theta in np.arange(-np.pi, np.pi, np.pi/100.0):
        rpose = create_2d_pose(np.cos(theta), np.sin(theta), 0, 'map')
        Visualization().add_line_marker([origin, rpose])
        Visualization().publish_markers()
        pt.update_pose(rpose)
        if theta > (3 * np.pi) / 4.0:
            assert pt.reached_waypoint() == 3
            assert pt.next_waypoint() is None
        elif theta > np.pi / 4.0:
            assert pt.reached_waypoint() == 2
            assert pt.next_waypoint() == 3
        elif theta > - np.pi / 4.0:
            assert pt.reached_waypoint() == 1
            assert pt.next_waypoint() == 2
        elif theta > - (3.0 * np.pi) / 4.0:
            assert pt.reached_waypoint() == 0
            assert pt.next_waypoint() == 1
        else:
            assert pt.reached_waypoint() is None
            assert pt.next_waypoint() == 0
