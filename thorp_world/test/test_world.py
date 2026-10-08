"""
thorp_world's core, without ROS: identity across detections, skills' effects, relations and persistence.

    cd ~/colcon_ws/thorp_world/src/thorp/thorp_world && PYTHONPATH=python python3 -m pytest test/test_world.py -v
"""

import math

import pytest

from thorp_world import (BELIEVED, DISTURBED, GRIPPER, MISSING, OBSERVED, OK, TRAY, Observation, Pose, World)

TABLE_SIZE = (0.55, 0.55, 0.02)
AT_TABLE = Pose(0.03, 0.0)  # 0.145 m from table()'s near edge


def table(x=0.45, y=0.0, yaw=0.0, size=TABLE_SIZE):
    return Observation('table', Pose(x, y, 0.475, yaw), size, 'white')


def cube(x, y, color='white'):
    return Observation('cube', Pose(x, y, 0.49), (0.025, 0.025, 0.025), color)


@pytest.fixture
def world():
    return World()


def test_objects_are_named_by_type_and_index(world):
    _, ids = world.observe_objects(table(), [cube(0.3, 0.0), cube(0.4, 0.1), cube(0.5, -0.1)], stamp=1.0)
    assert ids == ['cube 1', 'cube 2', 'cube 3']
    assert [t.id for t in world.tables()] == ['table 1']


def test_redetected_objects_keep_their_ids(world):
    world.observe_objects(table(), [cube(0.3, 0.0), cube(0.4, 0.1)], stamp=1.0)
    # seen again from elsewhere, slightly off and in another order
    _, ids = world.observe_objects(table(), [cube(0.41, 0.11), cube(0.31, 0.0)], stamp=2.0)
    assert ids == ['cube 2', 'cube 1']
    assert world.get('cube 2').stamp == 2.0


def test_an_object_far_from_any_known_one_is_new(world):
    world.observe_objects(table(), [cube(0.3, 0.0)], stamp=1.0)
    _, ids = world.observe_objects(table(), [cube(0.3, 0.0), cube(0.3, 0.2)], stamp=2.0)
    assert ids == ['cube 1', 'cube 2']


def test_an_object_identified_as_another_type_keeps_its_id_and_type(world):
    world.observe_objects(table(), [cube(0.3, 0.0)], stamp=1.0)
    star = Observation('star', Pose(0.31, 0.0, 0.49), (0.03, 0.03, 0.01), 'red')
    _, ids = world.observe_objects(table(), [star], stamp=2.0)
    assert ids == ['cube 1']
    seen = world.get('cube 1')
    assert (seen.type, seen.size, seen.color) == ('cube', (0.025, 0.025, 0.025), 'red')
    assert seen.pose.x == pytest.approx(0.31)


def test_redetected_tables_keep_their_ids(world):
    world.observe_tables([table()], stamp=1.0)
    # half a turn around is the same table, and a table 2 m away is another
    ids = world.observe_tables([table(0.47, 0.02, math.pi + 0.01), table(2.5, 0.0)], stamp=2.0)
    assert ids == ['table 1', 'table 2']
    assert world.get('table 1').pose.yaw == pytest.approx(0.01)


def test_a_partial_view_does_not_shrink_a_known_table(world):
    world.observe_tables([table()], stamp=1.0)
    world.observe_tables([table(0.35, 0.0, size=(0.55, 0.2, 0.02))], stamp=2.0)
    known = world.get('table 1')
    assert known.size == TABLE_SIZE and known.pose.x == pytest.approx(0.45)
    assert known.stamp == 2.0, "it was seen, though"


def test_a_fuller_view_replaces_a_partial_one(world):
    world.observe_tables([table(0.35, 0.0, size=(0.55, 0.2, 0.02))], stamp=1.0)
    world.observe_tables([table()], stamp=2.0)
    assert world.get('table 1').size == TABLE_SIZE


def test_objects_not_seen_where_the_view_covered_go_missing(world):
    world.observe_objects(table(), [cube(0.3, 0.0), cube(0.6, 0.2)], stamp=1.0)
    # a partial view of the near half only: the far cube may still be there
    near_half = table(0.31, 0.0, size=(0.27, 0.55, 0.02))
    world.observe_objects(near_half, [], stamp=2.0, robot_pose=AT_TABLE)
    assert world.get('cube 1').status == MISSING
    assert world.get('cube 1').stamp == 1.0, "a missing object keeps when it was last seen"
    assert world.get('cube 2').status == OK


def test_only_a_view_from_at_the_table_tells_objects_missing(world):
    world.observe_objects(table(), [cube(0.3, 0.0)], stamp=1.0)
    world.observe_objects(table(), [], stamp=2.0, robot_pose=Pose(-0.5, 0.0))
    assert world.get('cube 1').status == OK, "seen from 0.675 m away, it may just have been missed"
    world.observe_objects(table(), [], stamp=3.0)
    assert world.get('cube 1').status == OK, "nor from nowhere known"
    world.observe_objects(table(), [], stamp=4.0, robot_pose=AT_TABLE)
    assert world.get('cube 1').status == MISSING


def test_a_missing_object_seen_again_gets_its_id_back(world):
    world.observe_objects(table(), [cube(0.3, 0.0)], stamp=1.0)
    world.observe_objects(table(), [], stamp=2.0, robot_pose=AT_TABLE)
    _, ids = world.observe_objects(table(), [cube(0.3, 0.0)], stamp=3.0)
    assert ids == ['cube 1'] and world.get('cube 1').status == OK


def test_carried_objects_are_neither_matched_nor_missing(world):
    world.observe_objects(table(), [cube(0.3, 0.0), cube(0.4, 0.0)], stamp=1.0)
    world.mark_picked('cube 1', stamp=2.0)
    world.mark_stowed('cube 2', stamp=3.0, slot=4)
    # another cube where the picked one was is another object
    _, ids = world.observe_objects(table(), [cube(0.3, 0.0)], stamp=4.0, robot_pose=AT_TABLE)
    assert ids == ['cube 3']
    assert world.get('cube 1').support == GRIPPER and world.get('cube 1').status == OK
    assert world.get('cube 2').support == TRAY and world.get('cube 2').slot == 4


def test_skills_effects_are_beliefs(world):
    world.observe_objects(table(), [cube(0.3, 0.0)], stamp=1.0)
    world.mark_picked('cube 1', stamp=2.0)
    picked = world.get('cube 1')
    assert (picked.support, picked.pose, picked.provenance, picked.stamp) == (GRIPPER, None, BELIEVED, 2.0)

    world.mark_placed('cube 1', stamp=3.0, table_id='table 1', pose=Pose(0.2, 0.1, 0.49))
    placed = world.get('cube 1')
    assert placed.support == 'table 1' and placed.pose.x == pytest.approx(0.2)

    world.mark_disturbed('cube 1', stamp=4.0)
    assert world.get('cube 1').status == DISTURBED and world.get('cube 1').pose.x == pytest.approx(0.2)

    world.observe_objects(table(), [cube(0.21, 0.1)], stamp=5.0)
    seen = world.get('cube 1')
    assert (seen.status, seen.provenance) == (OK, OBSERVED)


def test_a_lost_object_is_missing_and_nowhere(world):
    world.observe_objects(table(), [cube(0.3, 0.0)], stamp=1.0)
    world.mark_picked('cube 1', stamp=2.0)
    world.mark_lost('cube 1', stamp=3.0)
    lost = world.get('cube 1')
    assert (lost.support, lost.pose, lost.status, lost.provenance) == (None, None, MISSING, BELIEVED)
    _, ids = world.observe_objects(table(), [cube(0.31, 0.0)], stamp=4.0)
    assert ids == ['cube 2'], "a lost object has no pose to be matched by"


def test_effects_on_unknown_entities_are_refused(world):
    world.observe_objects(table(), [cube(0.3, 0.0)], stamp=1.0)
    with pytest.raises(KeyError):
        world.mark_picked('cube 9', stamp=2.0)
    with pytest.raises(KeyError):
        world.mark_placed('cube 1', stamp=2.0, table_id='table 9', pose=Pose(0.2, 0.0))
    with pytest.raises(KeyError):
        world.mark_picked('table 1', stamp=2.0)


def test_the_robot_is_at_a_table_when_close_to_it(world):
    world.observe_tables([table(), table(0.45, 0.6)], stamp=1.0)
    # table 1's near edge is at x 0.175
    assert world.table_at(AT_TABLE) == 'table 1'
    assert world.table_at(Pose(-0.5, 0.0)) is None
    # between both, at the one docked at rather than the closest
    world.dock('table 2')
    assert world.table_at(Pose(0.03, 0.15)) == 'table 2'
    world.undock()
    assert world.table_at(Pose(0.03, 0.15)) == 'table 1'


def test_losing_an_object_not_in_the_gripper_changes_nothing(world):
    world.observe_objects(table(), [cube(0.3, 0.0)], stamp=1.0)
    world.mark_lost('cube 1', stamp=2.0)
    assert world.get('cube 1').support == 'table 1' and world.get('cube 1').pose is not None


def test_reach_is_measured_from_the_arm(world):
    world.observe_objects(table(), [cube(0.3, 0.0), cube(0.6, 0.2)], stamp=1.0)
    world.mark_picked('cube 2', stamp=2.0)
    arm = Pose(0.1, 0.0)
    assert world.within_reach('cube 1', arm, reach=0.3)
    assert not world.within_reach('cube 1', arm, reach=0.15)
    assert not world.within_reach('cube 2', arm, reach=10.0), "a carried object isn't on a table to reach"


def test_the_world_survives_a_save_and_a_load(world, tmp_path):
    world.observe_objects(table(), [cube(0.3, 0.0), cube(0.4, 0.1)], stamp=1.0)
    world.mark_stowed('cube 2', stamp=2.0, slot=1)
    world.dock('table 1')
    path = tmp_path / 'world.yaml'
    world.save(path)

    loaded = World()
    loaded.load(path)
    assert loaded.entities() == world.entities()
    assert loaded.docked_at == 'table 1'
    _, ids = loaded.observe_objects(table(), [cube(0.3, 0.0)], stamp=3.0)
    assert ids == ['cube 1'], "loaded entities are matched as any other"


def test_a_file_that_is_no_world_is_refused(world, tmp_path):
    world.observe_objects(table(), [cube(0.3, 0.0)], stamp=1.0)
    path = tmp_path / 'playground.sdf'
    path.write_text('<sdf version="1.9"><world name="default"/></sdf>\n')
    with pytest.raises(ValueError):
        world.load(path)
    assert [e.id for e in world.entities()] == ['table 1', 'cube 1'], 'and the world stays as it was'
