"""
What thorp_world knows, put for the agent: a summary to start each request from, and what a capability call changed.

Plain data in, text out, so it can be tested without ROS. A snapshot is what world.py reads from world/query and TF:
{'robot': {x, y, yaw} or None, 'shoulder': the arm's, {x, y} or None, 'at_table': a table id or None, 'docked':
whether the robot is docked at it, 'entities': {id: entity}}, each entity a dict with kind, type, color, pose ({x, y,
z, yaw} or None), size ([x, y, z] in its own frame, z up), support (a table id, 'gripper', 'tray' or None), slot,
status, provenance, inspected and reachable.
"""

import json
import math
from typing import Any, Dict, List, Optional

GRIPPER = 'gripper'
TRAY = 'tray'
OK = 'ok'
BELIEVED = 'believed'

# How far above the table a placed object is released, as the manipulation server rests it
PLACE_CLEARANCE = 0.01
# Smaller robot moves than these aren't worth telling
MOVED_DISTANCE = 0.05
MOVED_YAW = 0.1
# An object seen again farther than this from where it was known has moved; nearer, it's detection noise
OBJECT_MOVED_DISTANCE = 0.02
# How close to a compass point the robot's heading must be to be named by it
AXIS_TOLERANCE = math.radians(15)
# A table the robot faces, this close to its heading and this far at most, is the one the operator's near and far,
# left and right refer to, if the robot is at none
FACING_ANGLE = math.radians(45)
FACING_DISTANCE = 3.0
# The map's compass points, +x being east and +y north
COMPASS =((0.0, 'east'), (math.pi / 2.0, 'north'), (math.pi, 'west'), (-math.pi / 2.0, 'south'))


def summary(snapshot: Optional[Dict[str, Any]]) -> str:
    """What the world knows, a line per place: the robot, its gripper and tray, each table and its objects."""
    if snapshot is None:
        return 'thorp_world is not running: nothing known.'
    entities = snapshot['entities']
    tables = sorted(i for i, e in entities.items() if e['kind'] == 'table')
    objects = {i: e for i, e in entities.items() if e['kind'] == 'object'}

    at_table = snapshot.get('at_table')
    robot = snapshot.get('robot')
    where = _at_table(at_table, snapshot.get('docked'), robot) if at_table else 'at no table'
    lines = [f'Robot: {_robot(robot, snapshot.get("shoulder"))}; {where}.']
    viewpoint = _viewpoint(robot, at_table or _faced_table(robot, entities, tables), entities)
    if viewpoint:
        lines.append(viewpoint)
    held = sorted(i for i, e in objects.items() if e['support'] == GRIPPER)
    held_notes = ', '.join(i + _held(objects[i], at_table, entities.get(at_table)) for i in held)
    lines.append(f'Gripper: {held_notes or "empty"}.')
    stowed = sorted((i for i, e in objects.items() if e['support'] == TRAY),
                    key=lambda i: (objects[i]['slot'] is None, objects[i]['slot'] or 0, i))
    lines.append(f'Tray: {", ".join(_on_tray(i, objects[i]["slot"]) for i in stowed) if stowed else "empty"}.')
    for table in tables:
        known = entities[table]
        on_it = sorted(i for i, e in objects.items() if e['support'] == table)
        inspected = 'inspected' if known['inspected'] else 'not inspected'
        contents = ', '.join(_described(i, objects[i]) for i in on_it) if on_it else 'no objects known'
        lines.append(f'{table} at ({known["pose"]["x"]:.2f}, {known["pose"]["y"]:.2f}){_extent(known)}, {inspected}: '
                     f'{contents}.')
    lost = sorted(i for i, e in objects.items() if e['support'] is None)
    if lost:
        lines.append(f'Lost, whereabouts unknown: {", ".join(lost)}.')
    return '\n'.join(lines)


def changes(before: Optional[Dict[str, Any]], after: Optional[Dict[str, Any]]) -> List[str]:
    """What changed in the world between two snapshots, one sentence each; nothing if either is unknown."""
    if before is None or after is None:
        return []
    said = []
    if _moved(before.get('robot'), after.get('robot')):
        said.append(f'the robot moved to {_pose2d(after["robot"])}{_shoulder(after.get("shoulder"))}')
    # docked again on another side of the same table, the robot is at a new place too
    was_at, now_at = (_at_table(world['at_table'], world.get('docked'), world.get('robot'))
                      if world.get('at_table') else None for world in (before, after))
    if now_at != was_at:
        said.append('the robot is now ' + now_at if now_at else f'the robot left {before["at_table"]}')
    old, new = before['entities'], after['entities']
    for entity_id in sorted(new):
        entity = new[entity_id]
        if entity_id not in old:
            said.append(_found(entity_id, entity))
            continue
        was = old[entity_id]
        if entity['kind'] == 'table':
            if entity['inspected'] and not was['inspected']:
                said.append(f'{entity_id} inspected')
            continue
        if (entity['support'], entity['slot']) != (was['support'], was['slot']):
            table_id = after.get('at_table')
            held = _held(entity, table_id, new.get(table_id)) if entity['support'] == GRIPPER else ''
            said.append(f'{entity_id}: {_where(was)} -> {_where(entity)}{held or _at(entity)}')
        else:
            if entity['status'] != was['status']:
                status = 'seen again' if entity['status'] == OK else 'now ' + entity['status']
                said.append(f'{entity_id} {status}')
            if _object_moved(was, entity):
                said.append(f'{entity_id} moved to ({entity["pose"]["x"]:.2f}, {entity["pose"]["y"]:.2f})')
    for entity_id in sorted(set(old) - set(new)):
        said.append(f'{entity_id} forgotten')

    # an object found, moved or forgotten was told already; reach changes of the rest tell the robot moved
    stayed = {i for i in set(old) & set(new) if _where(old[i]) == _where(new[i])}
    reached = {i for i in stayed if new[i].get('reachable')}
    if reached != {i for i in stayed if old[i].get('reachable')}:
        said.append(f'within the arm\'s reach now: {", ".join(sorted(reached)) if reached else "nothing"}')
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
        outcome = f'failed ({reason})' if reason else 'failed'
    line = f'{subtree} {json.dumps(inputs or {})}: {outcome}'
    if result.get('world_changes'):
        line += '; world: ' + '; '.join(result['world_changes'])
    return line


def _robot(pose, shoulder):
    if pose is None:
        return 'pose unknown'
    yaw = pose['yaw']
    return (f'at {_pose2d(pose)}, facing {_direction(yaw)} with {_direction(yaw + math.pi / 2.0)} to its left'
            f'{_shoulder(shoulder)}')


def _shoulder(shoulder):
    return f", its arm's shoulder at ({shoulder['x']:.2f}, {shoulder['y']:.2f})" if shoulder else ''


def _at_table(table_id, docked, robot):
    """The table the robot is at, and the side of it it's on: facing the table, the side opposite its heading"""
    at = f'{"docked at" if docked else "at"} {table_id}'
    if robot is None:
        return at
    return f'{at}, on its {_compass_point(robot["yaw"] + math.pi)} side'


def _faced_table(robot, entities, tables):
    """The closest table the robot faces, if any"""
    if robot is None:
        return None
    faced = []
    for table in tables:
        pose = entities[table]['pose']
        distance = math.hypot(pose['x'] - robot['x'], pose['y'] - robot['y'])
        bearing = math.atan2(pose['y'] - robot['y'], pose['x'] - robot['x'])
        if distance <= FACING_DISTANCE and abs(math.remainder(bearing - robot['yaw'], 2.0 * math.pi)) <= FACING_ANGLE:
            faced.append((distance, table))
    return min(faced)[1] if faced else None


def _viewpoint(robot, table_id, entities):
    """
    A table's sides as the robot sees it from where it stands, named by compass: the near side faces the robot, and
    left and right are as seen looking across the table to its far side
    """
    if robot is None or table_id is None:
        return None
    pose = entities[table_id]['pose']
    far = math.atan2(pose['y'] - robot['y'], pose['x'] - robot['x'])
    near, left, right = (_compass_point(far + turn) for turn in (math.pi, math.pi / 2.0, -math.pi / 2.0))
    return (f"From where the robot stands, {table_id}'s near side is its {near} side, its far side the "
            f'{_compass_point(far)} one, its left the {left} one and its right the {right} one.')


def _compass_point(angle):
    """The compass point nearest to the direction of an angle"""
    return min(COMPASS, key=lambda point: abs(math.remainder(angle - point[0], 2.0 * math.pi)))[1]


def _pose2d(pose):
    return f'({pose["x"]:.2f}, {pose["y"]:.2f}), yaw {pose["yaw"]:.2f}'


def _direction(angle):
    """The compass point an angle points to, if it is close to one, or else its unit vector"""
    for axis, name in COMPASS:
        if abs(math.remainder(angle - axis, 2.0 * math.pi)) < AXIS_TOLERANCE:
            return name
    return f'({math.cos(angle):.2f}, {math.sin(angle):.2f})'


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
    return (f', spanning x {min(xs):.2f} to {max(xs):.2f} and y {min(ys):.2f} to {max(ys):.2f}, '
            f'top at z {_top(table):.3f}')


def _top(table):
    return table['pose']['z'] + table['size'][2] / 2.0


def _held(entity, table_id, table):
    """A note on an object in the gripper: its height, and the z that rests it on the table the robot is at"""
    size = entity.get('size')
    if not size:
        return ''
    note = f'{size[2] * 100:.1f} cm tall'
    if table and table.get('size') and 'z' in table['pose']:
        note += f'; to place it on {table_id}, its z is {_top(table) + size[2] / 2.0 + PLACE_CLEARANCE:.3f}'
    return f' ({note})'


def _at(entity):
    return f' at ({entity["pose"]["x"]:.2f}, {entity["pose"]["y"]:.2f})' if entity.get('pose') else ''


def _described(entity_id, entity):
    notes = [entity['color']] if entity.get('color') else []
    if entity.get('pose'):
        notes.append(_at(entity).strip())
    if entity.get('size'):
        notes.append(' x '.join(f'{side * 100:.1f}' for side in entity['size']) + ' cm')
    if entity['status'] != OK:
        notes.append(entity['status'])
    elif entity['provenance'] == BELIEVED:
        notes.append('believed, not seen there')
    if entity.get('reachable'):
        notes.append('within reach')
    return f'{entity_id} ({", ".join(notes)})' if notes else entity_id


def _on_tray(entity_id, slot):
    return entity_id if slot is None else f'{entity_id} (slot {slot})'


def _where(entity):
    if entity['support'] is None:
        return 'lost'
    if entity['support'] == TRAY and entity['slot'] is not None:
        return f'tray slot {entity["slot"]}'
    return entity['support']


def _found(entity_id, entity):
    if entity['kind'] == 'table':
        inspected = ', and inspected' if entity['inspected'] else ''
        return f'{entity_id} found at ({entity["pose"]["x"]:.2f}, {entity["pose"]["y"]:.2f}){inspected}'
    return f'{entity_id} found on {entity["support"] or "nothing known"}{_at(entity)}'
