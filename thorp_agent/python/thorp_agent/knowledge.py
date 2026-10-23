"""
What thorp_world knows, put for the agent: a summary to start each request from, and what a capability call changed.

Plain data in, text out, so it can be tested without ROS. A snapshot is what world.py reads from world/query and TF:
{'robot': {x, y, yaw} or None, 'at_table': a table id or None, 'entities': {id: entity}}, each entity a dict with kind,
type, color, pose ({x, y, z, yaw} or None), size ([x, y, z] in its own frame, z up), support (a table id, 'gripper',
'tray' or None), slot, status, provenance, inspected and reachable.
"""

import json
import math
from typing import Any, Dict, List, Optional

GRIPPER = 'gripper'
TRAY = 'tray'
OK = 'ok'
BELIEVED = 'believed'

# Smaller robot moves than these aren't worth telling
MOVED_DISTANCE = 0.05
MOVED_YAW = 0.1
# An object seen again farther than this from where it was known has moved; nearer, it's detection noise
OBJECT_MOVED_DISTANCE = 0.02
# How close to a compass point the robot's heading must be to be named by it
AXIS_TOLERANCE = math.radians(15)
# The map's compass points, +x being east and +y north
COMPASS = ((0.0, 'east'), (math.pi / 2.0, 'north'), (math.pi, 'west'), (-math.pi / 2.0, 'south'))


def summary(snapshot: Optional[Dict[str, Any]]) -> str:
    """What the world knows, a line per place: the robot, its gripper and tray, each table and its objects."""
    if snapshot is None:
        return 'thorp_world is not running: nothing known.'
    entities = snapshot['entities']
    tables = sorted(i for i, e in entities.items() if e['kind'] == 'table')
    objects = {i: e for i, e in entities.items() if e['kind'] == 'object'}

    at_table = snapshot.get('at_table')
    lines = ['Robot: {}; {}.'.format(_robot(snapshot.get('robot')),
                                     _at_table(at_table, snapshot.get('robot')) if at_table else 'at no table')]
    held = sorted(i for i, e in objects.items() if e['support'] == GRIPPER)
    lines.append('Gripper: {}.'.format(', '.join(i + _held(objects[i]) for i in held) if held else 'empty'))
    stowed = sorted((i for i, e in objects.items() if e['support'] == TRAY),
                    key=lambda i: (objects[i]['slot'] is None, objects[i]['slot'] or 0, i))
    lines.append('Tray: {}.'.format(', '.join(_on_tray(i, objects[i]['slot']) for i in stowed) if stowed else 'empty'))
    for table in tables:
        known = entities[table]
        on_it = sorted(i for i, e in objects.items() if e['support'] == table)
        lines.append('{} at ({:.2f}, {:.2f}){}, {}: {}.'.format(
            table, known['pose']['x'], known['pose']['y'], _extent(known),
            'inspected' if known['inspected'] else 'not inspected',
            ', '.join(_described(i, objects[i]) for i in on_it) if on_it else 'no objects known'))
    lost = sorted(i for i, e in objects.items() if e['support'] is None)
    if lost:
        lines.append('Lost, whereabouts unknown: {}.'.format(', '.join(lost)))
    return '\n'.join(lines)


def changes(before: Optional[Dict[str, Any]], after: Optional[Dict[str, Any]]) -> List[str]:
    """What changed in the world between two snapshots, one sentence each; nothing if either is unknown."""
    if before is None or after is None:
        return []
    said = []
    if _moved(before.get('robot'), after.get('robot')):
        said.append('the robot moved to {}'.format(_pose2d(after['robot'])))
    # docked again on another side of the same table, the robot is at a new place too
    was_at, now_at = (_at_table(world['at_table'], world.get('robot')) if world.get('at_table') else None
                      for world in (before, after))
    if now_at != was_at:
        said.append('the robot is now ' + now_at if now_at else 'the robot left {}'.format(before['at_table']))
    old, new = before['entities'], after['entities']
    for entity_id in sorted(new):
        entity = new[entity_id]
        if entity_id not in old:
            said.append(_found(entity_id, entity))
            continue
        was = old[entity_id]
        if entity['kind'] == 'table':
            if entity['inspected'] and not was['inspected']:
                said.append('{} inspected'.format(entity_id))
            continue
        if (entity['support'], entity['slot']) != (was['support'], was['slot']):
            held = _held(entity) if entity['support'] == GRIPPER else ''
            said.append('{}: {} -> {}{}'.format(entity_id, _where(was), _where(entity), held or _at(entity)))
        else:
            if entity['status'] != was['status']:
                status = 'seen again' if entity['status'] == OK else 'now ' + entity['status']
                said.append('{} {}'.format(entity_id, status))
            if _object_moved(was, entity):
                said.append('{} moved to ({:.2f}, {:.2f})'.format(entity_id, entity['pose']['x'], entity['pose']['y']))
    for entity_id in sorted(set(old) - set(new)):
        said.append('{} forgotten'.format(entity_id))

    # an object found, moved or forgotten was told already; reach changes of the rest tell the robot moved
    stayed = {i for i in set(old) & set(new) if _where(old[i]) == _where(new[i])}
    reached = {i for i in stayed if new[i].get('reachable')}
    if reached != {i for i in stayed if old[i].get('reachable')}:
        said.append('within the arm\'s reach now: {}'.format(', '.join(sorted(reached)) if reached else 'nothing'))
    return said


def call_line(subtree: str, inputs: Dict[str, Any], result: Dict[str, Any]) -> str:
    """One line for the operator per capability call: what was asked, how it went, and what it changed in the world."""
    outputs = result.get('outputs') or {}
    if result.get('succeeded'):
        outcome = 'succeeded'
    else:
        error = outputs.get('error')
        if isinstance(error, dict):
            reason = ': '.join(str(part) for part in (error.get('name') or error.get('code'), error.get('text'))
                               if part)
        else:
            # a bare code, as Nav2's tree nodes give, under its own key
            named = [outputs[key] for key in sorted(outputs) if key.endswith('error_name')]
            reason = (named[0] if named else error) or result.get('error')
        outcome = 'failed ({})'.format(reason) if reason else 'failed'
    line = '{} {}: {}'.format(subtree, json.dumps(inputs or {}), outcome)
    if result.get('world_changes'):
        line += '; world: ' + '; '.join(result['world_changes'])
    return line


def _robot(pose):
    if pose is None:
        return 'pose unknown'
    yaw = pose['yaw']
    return 'at {}, facing {} with {} to its left'.format(_pose2d(pose), _direction(yaw),
                                                         _direction(yaw + math.pi / 2.0))


def _at_table(table_id, robot):
    """The table the robot is docked at, and the side of it it's on: docked head-on, the side opposite its heading"""
    if robot is None:
        return 'at ' + table_id
    away = robot['yaw'] + math.pi
    side = min(COMPASS, key=lambda point: abs(math.remainder(away - point[0], 2.0 * math.pi)))[1]
    return 'at {}, on its {} side'.format(table_id, side)


def _pose2d(pose):
    return '({:.2f}, {:.2f}), yaw {:.2f}'.format(pose['x'], pose['y'], pose['yaw'])


def _direction(angle):
    """The compass point an angle points to, if it is close to one, or else its unit vector"""
    for axis, name in COMPASS:
        if abs(math.remainder(angle - axis, 2.0 * math.pi)) < AXIS_TOLERANCE:
            return name
    return '({:.2f}, {:.2f})'.format(math.cos(angle), math.sin(angle))


def _moved(was, now):
    if was is None or now is None:
        return False
    return (math.hypot(now['x'] - was['x'], now['y'] - was['y']) > MOVED_DISTANCE or
            abs(math.remainder(now['yaw'] - was['yaw'], 2.0 * math.pi)) > MOVED_YAW)


def _object_moved(was, now):
    if not was.get('pose') or not now.get('pose'):
        return False
    return math.hypot(now['pose']['x'] - was['pose']['x'], now['pose']['y'] - was['pose']['y']) > OBJECT_MOVED_DISTANCE


def _extent(table):
    """The map area a table covers and the height of its top, as far as its pose and size are known"""
    size, pose = table.get('size'), table['pose']
    if not size or 'z' not in pose:
        return ''
    c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
    xs = [pose['x'] + c * dx - s * dy for dx in (-size[0] / 2, size[0] / 2) for dy in (-size[1] / 2, size[1] / 2)]
    ys = [pose['y'] + s * dx + c * dy for dx in (-size[0] / 2, size[0] / 2) for dy in (-size[1] / 2, size[1] / 2)]
    return ', spanning x {:.2f} to {:.2f} and y {:.2f} to {:.2f}, top at z {:.3f}'.format(
        min(xs), max(xs), min(ys), max(ys), _top(table))


def _top(table):
    return table['pose']['z'] + table['size'][2] / 2.0


def _held(entity):
    """A note on an object in the gripper: its height"""
    size = entity.get('size')
    return ' ({:.1f} cm tall)'.format(size[2] * 100) if size else ''


def _at(entity):
    return ' at ({:.2f}, {:.2f})'.format(entity['pose']['x'], entity['pose']['y']) if entity.get('pose') else ''


def _described(entity_id, entity):
    notes = [entity['color']] if entity.get('color') else []
    if entity.get('pose'):
        notes.append(_at(entity).strip())
    if entity.get('size'):
        notes.append('{:.1f} x {:.1f} x {:.1f} cm'.format(*(side * 100 for side in entity['size'])))
    if entity['status'] != OK:
        notes.append(entity['status'])
    elif entity['provenance'] == BELIEVED:
        notes.append('believed, not seen there')
    if entity.get('reachable'):
        notes.append('within reach')
    return '{} ({})'.format(entity_id, ', '.join(notes)) if notes else entity_id


def _on_tray(entity_id, slot):
    return entity_id if slot is None else '{} (slot {})'.format(entity_id, slot)


def _where(entity):
    if entity['support'] is None:
        return 'lost'
    if entity['support'] == TRAY and entity['slot'] is not None:
        return 'tray slot {}'.format(entity['slot'])
    return entity['support']


def _found(entity_id, entity):
    if entity['kind'] == 'table':
        return '{} found at ({:.2f}, {:.2f}){}'.format(entity_id, entity['pose']['x'], entity['pose']['y'],
                                                       ', and inspected' if entity['inspected'] else '')
    return '{} found on {}{}'.format(entity_id, entity['support'] or 'nothing known', _at(entity))
