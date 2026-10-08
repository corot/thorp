"""
Between thorp_world's core types and its ROS messages.
"""

import json
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from thorp_msgs.msg import WorldEntity

from .entities import Entity, Observation, Pose


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_of(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def to_time(seconds):
    sec = math.floor(seconds)
    return Time(sec=int(sec), nanosec=int(round((seconds - sec) * 1e9)) % 1000000000)


def object_type(collision_object):
    """A detected object's template: its type key, or its id without the index detection appends to it."""
    if collision_object.type.key:
        return collision_object.type.key
    name, _, index = collision_object.id.rpartition(' ')
    return name if name and index.isdigit() else collision_object.id


def to_observation(collision_object, map_pose):
    """
    A detected table or object as an observation, given its pose already on the map frame. Its size and color come
    from type.db, as detections report them; the size, from its first primitive otherwise.
    """
    data = json.loads(collision_object.type.db) if collision_object.type.db else {}
    size = data.get('size')
    if size is None:
        dimensions = collision_object.primitives[0].dimensions if collision_object.primitives else []
        size = (list(dimensions) + [0.0] * 3)[:3]
    return Observation(object_type(collision_object), to_pose(map_pose), tuple(float(s) for s in size),
                       data.get('color', ''))


def to_pose(map_pose):
    """A planar pose of plain floats: tf2's transforms fill messages with numpy ones, that YAML can't save."""
    p = map_pose.pose
    return Pose(float(p.position.x), float(p.position.y), float(p.position.z), float(yaw_of(p.orientation)))


def to_msg(entity: Entity, map_frame):
    msg = WorldEntity(id=entity.id, kind=entity.kind, type=entity.type, color=entity.color,
                      support=entity.support or '', slot=-1 if entity.slot is None else entity.slot,
                      provenance=entity.provenance, stamp=to_time(entity.stamp), status=entity.status,
                      inspected=to_time(entity.inspected or 0.0))
    msg.size = Vector3(x=float(entity.size[0]), y=float(entity.size[1]), z=float(entity.size[2]))
    msg.has_pose = entity.pose is not None
    if entity.pose is not None:
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = map_frame
        msg.pose.header.stamp = msg.stamp
        msg.pose.pose.position.x = float(entity.pose.x)
        msg.pose.pose.position.y = float(entity.pose.y)
        msg.pose.pose.position.z = float(entity.pose.z)
        msg.pose.pose.orientation = quaternion_of(entity.pose.yaw)
    return msg
