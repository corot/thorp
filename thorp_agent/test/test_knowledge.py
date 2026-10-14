"""
What the agent is told of thorp_world: the summary each request starts with, and the changes each result carries.

    cd ~/colcon_ws/thorp/src/thorp/thorp_agent && python3 -m pytest test/test_knowledge.py -v
"""

import copy
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from thorp_agent import knowledge  # noqa: E402


def table(x, y, inspected=False):
    return {'kind': 'table', 'type': 'table', 'color': 'white', 'pose': {'x': x, 'y': y}, 'support': None,
            'slot': None, 'status': 'ok', 'provenance': 'observed', 'inspected': inspected, 'reachable': False}


def thing(support, color='red', status='ok', provenance='observed', slot=None, reachable=False, type_='cube'):
    pose = {'x': 0.3, 'y': 0.0} if support not in ('gripper', 'tray', None) else None
    return {'kind': 'object', 'type': type_, 'color': color, 'pose': pose, 'support': support, 'slot': slot,
            'status': status, 'provenance': provenance, 'inspected': False, 'reachable': reachable}


def playground():
    return {'at_table': 'table 1',
            'entities': {'table 1': table(0.45, 0.0, inspected=True), 'table 2': table(3.0, 2.0),
                         'cube 1': thing('table 1', 'gray', reachable=True),
                         'star 1': thing('table 1', 'red', status='missing'),
                         'square 1': thing('tray', 'green', slot=2),
                         'clover 1': thing('table 1', 'green', provenance='believed')}}


def test_the_summary_says_where_everything_is():
    said = knowledge.summary(playground())
    assert said.splitlines() == [
        'Robot: at table 1.',
        'Gripper: empty.',
        'Tray: square 1 (slot 2).',
        'table 1 at (0.45, 0.00), inspected: clover 1 (green, believed, not seen there), cube 1 (gray, within reach), '
        'star 1 (red, missing).',
        'table 2 at (3.00, 2.00), not inspected: no objects known.']


def test_lost_objects_and_the_gripper_are_summarized_too():
    world = playground()
    world['at_table'] = None
    world['entities']['cube 1'] = thing('gripper', 'gray')
    world['entities']['star 1'] = thing(None, 'red', status='missing')
    said = knowledge.summary(world)
    assert 'Robot: at no table.' in said and 'Gripper: cube 1.' in said
    assert 'Lost, whereabouts unknown: star 1.' in said


def test_without_the_world_nothing_is_known():
    assert 'not running' in knowledge.summary(None)
    assert knowledge.changes(None, playground()) == []


def test_a_pickup_moves_the_object_to_the_gripper():
    before = playground()
    after = copy.deepcopy(before)
    after['entities']['cube 1'] = thing('gripper', 'gray')
    assert knowledge.changes(before, after) == ['cube 1: table 1 -> gripper'], 'its reach goes without saying'


def test_stowing_names_the_tray_slot():
    before = playground()
    before['entities']['cube 1'] = thing('gripper', 'gray')
    after = copy.deepcopy(before)
    after['entities']['cube 1'] = thing('tray', 'gray', slot=3)
    assert knowledge.changes(before, after) == ['cube 1: gripper -> tray slot 3']


def test_approaching_a_table_tells_the_robot_is_there_and_what_it_reaches():
    before = playground()
    before['at_table'] = None
    before['entities']['cube 1']['reachable'] = False
    after = copy.deepcopy(before)
    after['at_table'] = 'table 1'
    after['entities']['cube 1']['reachable'] = True
    after['entities']['star 1']['status'] = 'ok'
    after['entities']['pentagon 1'] = thing('table 1', 'purple', type_='pentagon')
    assert knowledge.changes(before, after) == ['the robot is now at table 1', 'pentagon 1 found on table 1',
                                                'star 1 seen again', "within the arm's reach now: cube 1"]


def test_inspections_new_tables_and_missing_objects_are_changes():
    before = playground()
    after = copy.deepcopy(before)
    after['entities']['table 2']['inspected'] = True
    after['entities']['table 3'] = table(5.0, 1.0)
    after['entities']['clover 1']['status'] = 'missing'
    assert knowledge.changes(before, after) == ['clover 1 now missing', 'table 2 inspected',
                                                'table 3 found at (5.00, 1.00)']


def test_nothing_changed_says_nothing():
    assert knowledge.changes(playground(), playground()) == []
