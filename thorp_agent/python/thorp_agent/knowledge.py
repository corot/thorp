"""
What thorp_world knows, put for the agent: a summary to start each request from, and what a capability call changed.

Plain data in, text out, so it can be tested without ROS. A snapshot is what world.py reads from world/query:
{'at_table': a table id or None, 'entities': {id: entity}}, each entity a dict with kind, type, color, pose ({x, y} or
None), support (a table id, 'gripper', 'tray' or None), slot, status, provenance, inspected and reachable.
"""

from typing import Any, Dict, List, Optional

GRIPPER = 'gripper'
TRAY = 'tray'
OK = 'ok'
BELIEVED = 'believed'


def summary(snapshot: Optional[Dict[str, Any]]) -> str:
    """What the world knows, a line per place: the robot, its gripper and tray, each table and its objects."""
    if snapshot is None:
        return 'thorp_world is not running: nothing known.'
    entities = snapshot['entities']
    tables = sorted(i for i, e in entities.items() if e['kind'] == 'table')
    objects = {i: e for i, e in entities.items() if e['kind'] == 'object'}

    lines = ['Robot: at {}.'.format(snapshot['at_table']) if snapshot.get('at_table') else 'Robot: at no table.']
    held = sorted(i for i, e in objects.items() if e['support'] == GRIPPER)
    lines.append('Gripper: {}.'.format(', '.join(held) if held else 'empty'))
    stowed = sorted((i for i, e in objects.items() if e['support'] == TRAY),
                    key=lambda i: (objects[i]['slot'] is None, objects[i]['slot'] or 0, i))
    lines.append('Tray: {}.'.format(', '.join(_on_tray(i, objects[i]['slot']) for i in stowed) if stowed else 'empty'))
    for table in tables:
        known = entities[table]
        on_it = sorted(i for i, e in objects.items() if e['support'] == table)
        lines.append('{} at ({:.2f}, {:.2f}), {}: {}.'.format(
            table, known['pose']['x'], known['pose']['y'], 'inspected' if known['inspected'] else 'not inspected',
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
    if before.get('at_table') != after.get('at_table'):
        said.append('the robot is now at {}'.format(after['at_table']) if after.get('at_table') else
                    'the robot left {}'.format(before['at_table']))
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
            said.append('{}: {} -> {}'.format(entity_id, _where(was), _where(entity)))
        elif entity['status'] != was['status']:
            status = 'seen again' if entity['status'] == OK else 'now ' + entity['status']
            said.append('{} {}'.format(entity_id, status))
    for entity_id in sorted(set(old) - set(new)):
        said.append('{} forgotten'.format(entity_id))

    # an object found, moved or forgotten was told already; reach changes of the rest tell the robot moved
    stayed = {i for i in set(old) & set(new) if _where(old[i]) == _where(new[i])}
    reached = {i for i in stayed if new[i].get('reachable')}
    if reached != {i for i in stayed if old[i].get('reachable')}:
        said.append('within the arm\'s reach now: {}'.format(', '.join(sorted(reached)) if reached else 'nothing'))
    return said


def _described(entity_id, entity):
    notes = [entity['color']] if entity.get('color') else []
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
        return '{} found at ({:.2f}, {:.2f})'.format(entity_id, entity['pose']['x'], entity['pose']['y'])
    return '{} found on {}'.format(entity_id, entity['support'] or 'nothing known')
