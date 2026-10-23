"""
What the agent is told of thorp_world: the summary each request starts with, and the changes each result carries.

    cd ~/colcon_ws/thorp/src/thorp/thorp_agent && python3 -m pytest test/test_knowledge.py -v
"""

import copy
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from thorp_agent import knowledge  # noqa: E402


def table(x, y, inspected=False):
    return {'kind': 'table', 'type': 'table', 'color': 'white', 'pose': {'x': x, 'y': y, 'z': 0.475, 'yaw': 1.571},
            'size': [0.55, 0.5, 0.002], 'support': None, 'slot': None, 'status': 'ok', 'provenance': 'observed',
            'inspected': inspected, 'reachable': False}


def thing(support, color='red', status='ok', provenance='observed', slot=None, reachable=False, type_='cube'):
    pose = {'x': 0.3, 'y': 0.0, 'z': 0.49, 'yaw': 0.0} if support not in ('gripper', 'tray', None) else None
    return {'kind': 'object', 'type': type_, 'color': color, 'pose': pose, 'size': [0.025, 0.025, 0.026],
            'support': support, 'slot': slot, 'status': status, 'provenance': provenance, 'inspected': False,
            'reachable': reachable}


def playground():
    return {'robot': {'x': 0.03, 'y': 0.0, 'yaw': 0.01}, 'at_table': 'table 1',
            'entities': {'table 1': table(0.45, 0.0, inspected=True), 'table 2': table(3.0, 2.0),
                         'cube 1': thing('table 1', 'gray', reachable=True),
                         'star 1': thing('table 1', 'red', status='missing'),
                         'square 1': thing('tray', 'green', slot=2),
                         'clover 1': thing('table 1', 'green', provenance='believed')}}


def test_the_summary_says_where_everything_is():
    said = knowledge.summary(playground())
    assert said.splitlines() == [
        'Robot: at (0.03, 0.00), yaw 0.01, facing east with north to its left; at table 1, on its west side.',
        'Gripper: empty.',
        'Tray: square 1 (slot 2).',
        'table 1 at (0.45, 0.00), spanning x 0.20 to 0.70 and y -0.28 to 0.28, top at z 0.476, inspected: '
        'clover 1 (green, at (0.30, 0.00), 2.5 x 2.5 x 2.6 cm, believed, not seen there), '
        'cube 1 (gray, at (0.30, 0.00), 2.5 x 2.5 x 2.6 cm, within reach), '
        'star 1 (red, at (0.30, 0.00), 2.5 x 2.5 x 2.6 cm, missing).',
        'table 2 at (3.00, 2.00), spanning x 2.75 to 3.25 and y 1.72 to 2.28, top at z 0.476, not inspected: '
        'no objects known.']


def test_the_held_object_comes_with_its_height():
    world = playground()
    world['entities']['cube 1'] = thing('gripper', 'gray')
    assert 'Gripper: cube 1 (2.6 cm tall).' in knowledge.summary(world)


def test_the_arms_shoulder_is_placed_on_the_map_as_the_robot_moves():
    before = playground()
    before['shoulder'] = {'x': 0.1, 'y': 0.03}
    assert knowledge.summary(before).splitlines()[0] == (
        "Robot: at (0.03, 0.00), yaw 0.01, facing east with north to its left, its arm's shoulder at (0.10, 0.03); "
        'at table 1, on its west side.')
    after = copy.deepcopy(before)
    after['robot'] = {'x': 0.9, 'y': 0.0, 'yaw': math.pi}
    after['shoulder'] = {'x': 0.8, 'y': -0.03}
    assert knowledge.changes(before, after)[0] == \
        "the robot moved to (0.90, 0.00), yaw 3.14, its arm's shoulder at (0.80, -0.03)"


def test_a_heading_off_the_compass_points_is_given_as_vectors():
    world = playground()
    world['robot'] = {'x': 1.0, 'y': 2.0, 'yaw': math.pi / 4}
    world['at_table'] = None
    assert knowledge.summary(world).splitlines()[0] == \
        'Robot: at (1.00, 2.00), yaw 0.79, facing (0.71, 0.71) with (-0.71, 0.71) to its left; at no table.'
    world['at_table'] = 'table 1'
    world['robot'] = None
    assert knowledge.summary(world).splitlines()[0] == 'Robot: pose unknown; at table 1.'


def test_lost_objects_and_the_gripper_are_summarized_too():
    world = playground()
    world['at_table'] = None
    world['entities']['cube 1'] = thing('gripper', 'gray')
    world['entities']['star 1'] = thing(None, 'red', status='missing')
    said = knowledge.summary(world)
    assert 'yaw 0.01, facing east with north to its left; at no table.' in said
    assert 'Gripper: cube 1 (2.6 cm tall).' in said
    assert 'Lost, whereabouts unknown: star 1.' in said


def test_without_the_world_nothing_is_known():
    assert 'not running' in knowledge.summary(None)
    assert knowledge.changes(None, playground()) == []


def test_a_pickup_moves_the_object_to_the_gripper():
    before = playground()
    after = copy.deepcopy(before)
    after['entities']['cube 1'] = thing('gripper', 'gray')
    assert knowledge.changes(before, after) == ['cube 1: table 1 -> gripper (2.6 cm tall)'], 'its reach goes unsaid'


def test_a_place_tells_where_the_object_went():
    before = playground()
    before['entities']['cube 1'] = thing('gripper', 'gray')
    after = copy.deepcopy(before)
    after['entities']['cube 1'] = thing('table 1', 'gray')
    after['entities']['cube 1']['pose'].update(x=0.25, y=0.12)
    assert knowledge.changes(before, after) == ['cube 1: gripper -> table 1 at (0.25, 0.12)']


def test_stowing_names_the_tray_slot():
    before = playground()
    before['entities']['cube 1'] = thing('gripper', 'gray')
    after = copy.deepcopy(before)
    after['entities']['cube 1'] = thing('tray', 'gray', slot=3)
    assert knowledge.changes(before, after) == ['cube 1: gripper -> tray slot 3']


def test_approaching_a_table_tells_the_robot_is_there_and_what_it_reaches():
    before = playground()
    before['robot'] = {'x': -0.5, 'y': 0.0, 'yaw': 0.0}
    before['at_table'] = None
    before['entities']['cube 1']['reachable'] = False
    after = copy.deepcopy(before)
    after['robot'] = {'x': 0.03, 'y': 0.0, 'yaw': 0.01}
    after['at_table'] = 'table 1'
    after['entities']['cube 1']['reachable'] = True
    after['entities']['star 1']['status'] = 'ok'
    after['entities']['pentagon 1'] = thing('table 1', 'purple', type_='pentagon')
    assert knowledge.changes(before, after) == [
        'the robot moved to (0.03, 0.00), yaw 0.01', 'the robot is now at table 1, on its west side',
        'pentagon 1 found on table 1 at (0.30, 0.00)', 'star 1 seen again', "within the arm's reach now: cube 1"]


def test_docking_at_another_side_of_the_table_tells_the_side():
    before = playground()
    after = copy.deepcopy(before)
    after['robot'] = {'x': 0.45, 'y': 0.5, 'yaw': -math.pi / 2}
    assert knowledge.changes(before, after) == [
        'the robot moved to (0.45, 0.50), yaw -1.57', 'the robot is now at table 1, on its north side']
    assert knowledge.summary(after).splitlines()[0] == \
        'Robot: at (0.45, 0.50), yaw -1.57, facing south with east to its left; at table 1, on its north side.'


def test_small_robot_moves_go_unsaid():
    before = playground()
    after = copy.deepcopy(before)
    after['robot'] = {'x': 0.06, 'y': 0.01, 'yaw': 0.05}
    assert knowledge.changes(before, after) == []


def test_inspections_new_tables_and_missing_objects_are_changes():
    before = playground()
    after = copy.deepcopy(before)
    after['entities']['table 2']['inspected'] = True
    after['entities']['table 3'] = table(5.0, 1.0)
    after['entities']['table 4'] = table(5.0, 3.0, inspected=True)
    after['entities']['clover 1']['status'] = 'missing'
    assert knowledge.changes(before, after) == ['clover 1 now missing', 'table 2 inspected',
                                                'table 3 found at (5.00, 1.00)',
                                                'table 4 found at (5.00, 3.00), and inspected']


def test_an_object_seen_again_elsewhere_on_its_table_moved():
    before = playground()
    after = copy.deepcopy(before)
    after['entities']['cube 1']['pose'].update(x=0.34, y=-0.05)
    after['entities']['star 1']['pose'].update(x=0.31)
    assert knowledge.changes(before, after) == ['cube 1 moved to (0.34, -0.05)'], 'within 2 cm, detection noise'


def test_nothing_changed_says_nothing():
    assert knowledge.changes(playground(), playground()) == []


def test_a_call_line_tells_the_call_its_outcome_and_the_world_changes():
    result = {'succeeded': True, 'outputs': {}, 'world_changes': ['star 1: table 1 -> gripper']}
    assert knowledge.call_line('pickup_object', {'object_name': 'star 1'}, result) == \
        'pickup_object {"object_name": "star 1"}: succeeded; world: star 1: table 1 -> gripper'


def test_a_failed_call_line_says_why():
    failed = {'succeeded': False, 'world_changes': [],
              'outputs': {'error': {'code': -230, 'name': 'NOT_AT_TABLE', 'text': "'cube 1' is on 'table 2'"}}}
    assert knowledge.call_line('pickup_object', {'object_name': 'cube 1'}, failed) == \
        'pickup_object {"object_name": "cube 1"}: failed (NOT_AT_TABLE: \'cube 1\' is on \'table 2\')'
    driving = {'succeeded': False, 'outputs': {'nav_error': 104, 'nav_error_name': 'PATIENCE_EXCEEDED'}}
    assert knowledge.call_line('reach_table', {}, driving) == 'reach_table {}: failed (PATIENCE_EXCEEDED)'
    refused = {'succeeded': False, 'refused': True, 'error': "missing input 'pose'"}
    assert knowledge.call_line('reach_table', {}, refused) == "reach_table {}: failed (missing input 'pose')"
