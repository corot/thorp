"""
Geometry on tables, for matching observations and deriving relations.
"""

import math

from .entities import Entity, Pose


def normalize_table_yaw(yaw):
    """A table is the same rotated half a turn, so its yaw is kept within (-pi/2, pi/2]."""
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    if yaw <= -math.pi / 2.0:
        yaw += math.pi
    elif yaw > math.pi / 2.0:
        yaw -= math.pi
    return yaw


def to_table_frame(table: Entity, x, y):
    """A map point on the table's frame: centered on it, x along its longest side."""
    dx, dy = x - table.pose.x, y - table.pose.y
    cos, sin = math.cos(table.pose.yaw), math.sin(table.pose.yaw)
    return cos * dx + sin * dy, -sin * dx + cos * dy


def distance_to_table(table: Entity, x, y):
    """Planar distance from a map point to the table's footprint; zero on it."""
    tx, ty = to_table_frame(table, x, y)
    return math.hypot(max(abs(tx) - table.size[0] / 2.0, 0.0), max(abs(ty) - table.size[1] / 2.0, 0.0))


def on_table(table: Entity, x, y, margin=0.0):
    """Whether a map point is on the table's footprint, enlarged by margin."""
    return distance_to_table(table, x, y) <= margin


def planar_distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def grasp_point(entity: Entity):
    """Where the arm reaches for an object to pick it: its top center."""
    return Pose(entity.pose.x, entity.pose.y, entity.pose.z + entity.size[2] / 2.0)


def reach_distance(point, shoulder, max_distance, max_height):
    """
    How far a point is from the arm's shoulder, if the arm reaches it: no farther than max_distance, and no more than
    max_height above or below the shoulder, as the manipulation server measures its targets; None otherwise.
    """
    distance = math.dist((point.x, point.y, point.z), (shoulder.x, shoulder.y, shoulder.z))
    return distance if distance <= max_distance and abs(point.z - shoulder.z) <= max_height else None
