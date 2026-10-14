"""
capabilities.yaml turned into tool descriptions an LLM can call.

The yaml is the single source of truth: thorp_bt_cpp's drift test already holds it to the trees, so anything an agent
is offered here is a tree that exists, takes the inputs named, and returns the outputs promised. Nothing in this module
imports ROS or langchain, so what an agent will be shown can be checked without either.
"""

from typing import Any, Dict, List

import yaml

# How a capability's C++ type is described to the agent. Poses are offered as strings, as the ';' form is far easier
# for a model to emit without mistakes, but also taken in the object form calls return, as bt_server's seeding
# accepts both and values are passed back verbatim.
POSE = 'geometry_msgs::msg::PoseStamped'
# Collision objects come in lists, as their ids and colors
COLLISION_OBJECT = 'moveit_msgs::msg::CollisionObject'

TYPES = {
    'bool': ('boolean', None),
    'int': ('integer', None),
    'unsigned int': ('integer', None),
    'unsigned': ('integer', None),
    'uint16_t': ('integer', None),
    'uint32_t': ('integer', None),
    'unsigned long': ('integer', None),
    'float': ('number', None),
    'double': ('number', None),
    'std::string': ('string', None),
    'string': ('string', None),
    POSE: ('pose', "a pose as 'x;y;yaw;frame' or 'x;y;z;roll;pitch;yaw;frame', e.g. '1.5;0.0;1.57;map', or as "
                   'an earlier call returned it. Frames other than map are rarely what you want'),
    'std::vector<' + POSE + '>': ('array-of-pose', "poses, each as 'x;y;yaw;frame' or as an earlier call "
                                                   'returned it'),
    'std::vector<' + COLLISION_OBJECT + '>': ('array-of-object', 'one name and color per object'),
    'std::map<std::string, unsigned int>': ('counts', None),
    'std::map<std::string, uint32_t>': ('counts', None),
    'std::vector<unsigned int>': ('array-of-integer', None),
    'std::vector<uint32_t>': ('array-of-integer', None),
}

# Every kind TYPES can name. tools.py has to render each one, and a kind with no rendering would silently become a
# string, which the model then sends where an object was wanted.
KINDS = {'boolean', 'integer', 'number', 'string', 'object', 'pose',
         'array-of-string', 'array-of-integer', 'array-of-object', 'array-of-pose', 'counts'}


def load(path: str) -> Dict[str, Any]:
    with open(path) as f:
        return yaml.safe_load(f)


def offered(document: Dict[str, Any]) -> Dict[str, Any]:
    """
    The capabilities an agent may call.

    Apps are withheld: they run until something stops them, and a model that calls one has no way to get the robot
    back. Anything marked blocked is withheld too, since offering a tool that is known not to work only teaches the
    model that its tools are unreliable.
    """
    return {name: spec
            for name, spec in (document.get('capabilities') or {}).items()
            if spec.get('kind', 'capability') != 'app' and spec.get('status') != 'blocked'}


def _describe(field: str, meta: Dict[str, Any]) -> str:
    kind, hint = TYPES.get(meta.get('type', ''), ('string', None))
    parts = [meta.get('description', field), '({})'.format(kind)]
    if hint:
        parts.append('-- ' + hint)
    return ' '.join(parts)


def _errors(spec: Dict[str, Any]) -> str:
    """The errors this capability fails with when the world says it can't do what it was asked, or nothing."""
    errors = spec.get('errors') or {}
    if not errors:
        return ''
    return ' It fails with ' + ', '.join('{} ({})'.format(code, ' '.join(str(meaning).split()))
                                         for code, meaning in sorted(errors.items())) + '.'


def tool_spec(name: str, spec: Dict[str, Any]) -> Dict[str, Any]:
    """
    One capability as {name, description, args}, with args keyed by input name.

    The description carries the outputs as well as the inputs. Each call builds a fresh tree with an empty blackboard,
    so nothing carries over between calls: an agent that wants a detected table in the next call has to pass the value
    it was given back in. Saying so in every tool's description is the difference between an agent that chains calls
    and one that calls detect_table repeatedly wondering why the next step fails.
    """
    text = ' '.join((spec.get('description') or name).split())

    outputs = spec.get('outputs') or {}
    if outputs:
        listed = []
        for key, meta in sorted(outputs.items()):
            when = meta.get('when', 'success')
            qualifier = {'failure': ' (only when it fails)', 'maybe': ' (not always)'}.get(when, '')
            listed.append('{}: {}{}'.format(key, meta.get('description', ''), qualifier))
        text += ' Returns ' + '; '.join(listed) + '.'
        text += ' These values are yours to pass back into later calls.'
    text += _errors(spec)

    args = {}
    for field, meta in (spec.get('inputs') or {}).items():
        args[field] = {'type': meta.get('type', ''),
                       'optional': bool(meta.get('optional')),
                       'description': _describe(field, meta)}
    return {'name': name, 'description': text, 'args': args,
            'outputs': sorted(outputs), 'stack': spec.get('stack', [])}


def specs_from(document: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Every capability an agent may call, as a tool spec. One place, so nothing renders twice."""
    return [tool_spec(name, spec) for name, spec in sorted(offered(document).items())]


def tool_specs(path: str) -> List[Dict[str, Any]]:
    return specs_from(load(path))
