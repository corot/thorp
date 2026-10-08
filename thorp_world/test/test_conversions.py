"""
Between thorp_world's core types and its ROS messages, as detections fill them.
"""

import json
import math

import numpy as np
import pytest

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from thorp_msgs.msg import WorldEntity
from thorp_world import GRIPPER, OBJECT, TABLE, Entity, Pose
from thorp_world.conversions import (object_type, quaternion_of, to_msg, to_observation, to_seconds, to_time,
                                     yaw_of)


def detected(name, size=(0.03, 0.02, 0.01), color='red', db=True):
    obj = CollisionObject(id=name)
    if db:
        obj.type.db = json.dumps({'size': list(size), 'color': color})
    return obj


def map_pose(x, y, z, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
    pose.pose.orientation = quaternion_of(yaw)
    return pose


def test_an_object_type_is_its_template():
    assert object_type(detected('cube 3')) == 'cube'
    assert object_type(detected('table')) == 'table'
    keyed = detected('cube 3')
    keyed.type.key = 'box'
    assert object_type(keyed) == 'box', "a type key, when there's one, prevails over the name"


def test_size_and_color_come_from_the_detection_metadata():
    obs = to_observation(detected('star 1', (0.04, 0.03, 0.01), 'purple'), map_pose(0.3, 0.1, 0.49, 0.5))
    assert (obs.type, obs.size, obs.color) == ('star', (0.04, 0.03, 0.01), 'purple')
    assert (obs.pose.x, obs.pose.y, obs.pose.z) == (0.3, 0.1, 0.49)
    assert obs.pose.yaw == pytest.approx(0.5)


def test_without_metadata_the_size_is_the_first_primitive_one():
    box = detected('table', db=False)
    box.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.55, 0.5, 0.02]))
    obs = to_observation(box, map_pose(0.45, 0.0, 0.47, 0.0))
    assert obs.size == (0.55, 0.5, 0.02) and obs.color == ''


def test_yaw_and_time_round_trip():
    for yaw in (0.0, 1.2, -2.9, math.pi / 2):
        assert yaw_of(quaternion_of(yaw)) == pytest.approx(yaw)
    assert to_seconds(to_time(123.456789)) == pytest.approx(123.456789)


def test_a_carried_entity_has_no_pose():
    msg = to_msg(Entity('cube 1', OBJECT, 'cube', (0.025,) * 3, 'white', support=GRIPPER, stamp=5.0), 'map')
    assert not msg.has_pose and msg.support == WorldEntity.GRIPPER and msg.slot == -1
    assert to_seconds(msg.stamp) == pytest.approx(5.0)


def test_an_entity_on_a_table_has_a_map_pose():
    msg = to_msg(Entity('cube 1', OBJECT, 'cube', (0.025,) * 3, 'white', Pose(0.3, 0.1, 0.49, 1.0), 'table 1'), 'map')
    assert msg.has_pose and msg.pose.header.frame_id == 'map'
    assert msg.pose.pose.position.x == pytest.approx(0.3) and yaw_of(msg.pose.pose.orientation) == pytest.approx(1.0)


def test_a_table_never_inspected_has_a_zero_stamp_for_it():
    table = Entity('table 1', TABLE, 'table', (0.55, 0.55, 0.02), 'white', Pose(0.45, 0.0, 0.47, 0.0))
    assert to_seconds(to_msg(table, 'map').inspected) == 0.0
    table.inspected = 7.5
    assert to_seconds(to_msg(table, 'map').inspected) == pytest.approx(7.5)


def test_poses_are_plain_floats_even_from_numpy_ones():
    # as tf2's do_transform_pose_stamped leaves them
    pose = map_pose(np.float64(0.3), np.float64(0.1), np.float64(0.49), 0.5)
    obs = to_observation(detected('cube 1'), pose)
    assert all(type(v) is float for v in (obs.pose.x, obs.pose.y, obs.pose.z, obs.pose.yaw))
