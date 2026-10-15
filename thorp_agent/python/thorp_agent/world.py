"""
What is true of the robot right now, read rather than remembered.

Anything that moves the robot is a behavior tree, declared in capabilities.yaml and run through bt_server. What is here
is the other half: observations, which touch nothing and answer what holds now, and what thorp_world knows of the
tables and objects around. They ride on every capability result, so knowing where things stand costs the agent no call
of its own. Whether the gripper is closed on something isn't read: only closing it tells, as the trees' gripper check
does, and an observation changes nothing.
"""

import math

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from thorp_msgs.srv import QueryWorld

from .runner import wait

PLANNING_SCENE = '/get_planning_scene'
TARGET_TOPIC = 'target_object_pose'
WORLD_QUERY = '/world/query'
MAP_FRAME = 'map'
ROBOT_FRAME = 'base_footprint'

# How recently the tracker must have published for the target to count as still in view. The follower gives up after
# 5 s of silence, so this is the shorter of the two answers.
TARGET_FRESH_FOR = 3.0


class World(object):
    """
    Reads the robot's situation on demand.

    Every reading is optional: the agent talks to whatever stack happens to be running, and a missing service means
    that fact is unknown, not that the run failed. An absent key says "couldn't look" and a present one says what was
    seen, which are different answers and the agent should be able to tell them apart.
    """

    def __init__(self, node, scene_service=PLANNING_SCENE, target_topic=TARGET_TOPIC, world_query=WORLD_QUERY):
        self.node = node
        self.scene = node.create_client(GetPlanningScene, scene_service)
        self.world_query = node.create_client(QueryWorld, world_query)
        # listening from the start, so the robot's pose is there by the first request
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.target_heard = None
        node.create_subscription(PoseStamped, target_topic, self._target_heard, 1)

    def observe(self):
        seen = {}
        scene = self._planning_scene()
        if scene is not None:
            attached = scene.robot_state.attached_collision_objects
            seen['holding'] = attached[0].object.id if attached else None
            seen['scene_objects'] = sorted(o.id for o in scene.world.collision_objects)
        seen['target_in_view'] = self._target_fresh(TARGET_FRESH_FOR)
        return seen

    def known(self):
        """What thorp_world knows now, as knowledge.py takes it; None if it isn't running."""
        if not self.world_query.wait_for_service(timeout_sec=2.0):
            return None
        response = wait(self.world_query.call_async(QueryWorld.Request()), 5.0)
        if response is None or not response.success:
            self.node.get_logger().warn('cannot query thorp_world')
            return None
        entities = {e.id: _entity(e, reachable) for e, reachable in zip(response.entities, response.reachable)}
        return {'robot': self._robot_pose(), 'at_table': response.at_table or None, 'entities': entities}

    def _robot_pose(self):
        """Where the robot is on the map, as {x, y, yaw}; None if the transform isn't there"""
        try:
            transform = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, Time())
        except TransformException:
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        return {'x': round(t.x, 3), 'y': round(t.y, 3), 'yaw': round(_yaw(q), 3)}

    def _target_heard(self, _):
        self.target_heard = self.node.get_clock().now()

    def _target_fresh(self, within):
        if self.target_heard is None:
            return False
        return (self.node.get_clock().now() - self.target_heard).nanoseconds / 1e9 < within

    def _planning_scene(self):
        if not self.scene.wait_for_service(timeout_sec=2.0):
            return None
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                          PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        response = wait(self.scene.call_async(request), 5.0)
        if response is None:
            self.node.get_logger().warn('cannot read the planning scene')
            return None
        return response.scene


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _entity(entity, reachable):
    position = entity.pose.pose.position
    pose = {'x': round(position.x, 3), 'y': round(position.y, 3), 'z': round(position.z, 3),
            'yaw': round(_yaw(entity.pose.pose.orientation), 3)} if entity.has_pose else None
    return {'kind': entity.kind, 'type': entity.type, 'color': entity.color, 'pose': pose,
            'size': [round(entity.size.x, 3), round(entity.size.y, 3), round(entity.size.z, 3)],
            'support': entity.support or None, 'slot': entity.slot if entity.slot >= 0 else None,
            'status': entity.status, 'provenance': entity.provenance,
            'inspected': bool(entity.inspected.sec or entity.inspected.nanosec), 'reachable': bool(reachable)}
