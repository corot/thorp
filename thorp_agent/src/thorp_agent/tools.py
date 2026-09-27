"""
Tool specs turned into langchain tools ROSA can call.

Split from capabilities.py so that what the agent is offered can be tested without langchain
or ROS installed, which is most of what is worth testing here.
"""

import json
from typing import Any, Callable, Dict, List, Optional

from langchain_core.tools import StructuredTool
from pydantic import BaseModel, Field, create_model

from .capabilities import TYPES


class Pose(BaseModel):
    """A pose as bt_server emits one, which is also a shape seeding accepts back."""

    x: float
    y: float
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    frame: str = "map"


class Table(BaseModel):
    """
    A detected surface, spelled out rather than left as a free-form object.

    This is the value the whole chain turns on: detect_table returns it and three capabilities
    take it back. Declaring the fields gives the model a shape to copy instead of a blob to
    paraphrase, and keeps the schema acceptable to providers that reject an object with no
    declared properties.
    """

    name: str
    width: float
    depth: float
    height: float
    pose: Pose


PY_TYPES = {
    "boolean": bool,
    "integer": int,
    "number": float,
    "string": str,
    "object": Dict[str, Any],
    "array-of-string": List[str],
    "array-of-integer": List[int],
    "array-of-object": List[Dict[str, Any]],
    "table": Table,
    "counts": Dict[str, int],
}


def plain(value):
    """Back to what json.dumps takes: pydantic models in, dicts and lists out."""
    if isinstance(value, BaseModel):
        return value.model_dump()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, list):
        return [plain(v) for v in value]
    return value


def _args_model(spec: Dict[str, Any]):
    fields = {}
    for field, meta in spec["args"].items():
        kind, _ = TYPES.get(meta["type"], ("string", None))
        if meta.get("optional"):
            fields[field] = (Optional[PY_TYPES.get(kind, str)],
                             Field(default=None, description=meta["description"]))
        else:
            # a goal missing a required input aborts as soon as the tree reads it
            fields[field] = (PY_TYPES.get(kind, str), Field(description=meta["description"]))
    # an empty model even with no inputs: without one, langchain derives the schema from
    # invoke(**kwargs) and offers the model a `kwargs` argument
    return create_model(spec["name"] + "_args", **fields)


def build(specs: List[Dict[str, Any]],
          call: Callable[..., Dict[str, Any]],
          timeouts: Optional[Dict[str, float]] = None) -> List[StructuredTool]:
    """
    One tool per capability, each closing over `call(subtree, inputs, output_keys, timeout)`.

    `call` is injected rather than imported so the ROS client stays out of this module: a test
    passes a recorder, the node passes a SubtreeRunner.
    """
    timeouts = timeouts or {}
    tools = []
    for spec in specs:
        tools.append(_one(spec, call, timeouts.get(spec["name"], 300.0)))
    return tools


OBSERVE = ("What is true of the robot right now: what the gripper holds, what the planning "
           "scene contains, whether a target is in view. Reads the robot and changes nothing, "
           "so it is safe at any time. The same reading comes back on every other tool's "
           "result, so this is for before the first one, or after somebody has been at the "
           "robot by hand.")


def observer(observe: Callable[[], Dict[str, Any]], name: str = "robot_status") -> StructuredTool:
    """
    The one tool that is not a behavior tree.

    Every capability is a subtree bt_server runs, held to capabilities.yaml by the drift test.
    An observation runs nothing and changes nothing, so it has no tree to be held to, and
    putting one in that file would mean describing an interface that doesn't exist.
    """

    def invoke():
        return json.dumps(observe(), sort_keys=True)

    invoke.__name__ = name
    return StructuredTool.from_function(func=invoke, name=name, description=OBSERVE)


def _one(spec: Dict[str, Any], call, timeout: float) -> StructuredTool:
    name = spec["name"]
    outputs = sorted(spec.get("outputs") or [])

    def invoke(**kwargs):
        # an optional input left out stays out, so the tree falls back to its own value
        inputs = {k: v for k, v in plain(kwargs).items() if v is not None}
        result = call(name, inputs=inputs, output_keys=outputs, timeout=timeout)
        # json rather than a python repr: the model reads this back and has to be able to
        # quote values out of it verbatim into the next call.
        return json.dumps(result, sort_keys=True)

    invoke.__name__ = name
    model = _args_model(spec)
    return StructuredTool.from_function(
        func=invoke, name=name, description=spec["description"], args_schema=model)
