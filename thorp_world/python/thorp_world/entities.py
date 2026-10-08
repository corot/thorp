"""
The things thorp_world knows about, tables and the objects on them, in the map frame.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

# kinds
TABLE = 'table'
OBJECT = 'object'

# supports other than a table
GRIPPER = 'gripper'
TRAY = 'tray'

# provenances
OBSERVED = 'observed'  # a detection saw it so
BELIEVED = 'believed'  # a skill left it so, as far as it knows

# statuses
OK = 'ok'
DISTURBED = 'disturbed'  # a skill touched it without the intended result, so it may have moved
MISSING = 'missing'  # not seen where it was, from at its table


@dataclass
class Pose:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


@dataclass
class Observation:
    """A table or an object as a detection reports it, already in the map frame."""
    type: str
    pose: Pose
    size: Tuple[float, float, float]  # tables have x along their longest side
    color: str = ''


@dataclass
class Entity:
    id: str
    kind: str
    type: str
    size: Tuple[float, float, float]
    color: str = ''
    pose: Optional[Pose] = None  # none while carried, as the robot moves it
    support: Optional[str] = None  # a table id, GRIPPER or TRAY
    slot: Optional[int] = None  # on the tray
    provenance: str = OBSERVED
    stamp: float = 0.0  # when last observed, or last changed by a skill
    status: str = OK
