"""
What is true of the robot right now, read rather than remembered.

Anything that moves the robot is a behavior tree, declared in capabilities.yaml and run through bt_server. What is here
is the other half: observations, which touch nothing and answer whether the states that file names actually hold. They
ride on every capability result, so knowing where things stand costs the agent no call of its own.

One state in the glossary has no reading behind it. Nothing reports "the robot is parked against a table", so
at_a_table stays an assumption from whatever approach_table last did, and anybody who pushes the robot invalidates it
silently. Whether the gripper is closed on something isn't read either: only closing it tells, as the trees' gripper
check does, and an observation changes nothing.
"""

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

from .runner import wait

PLANNING_SCENE = '/get_planning_scene'
TARGET_TOPIC = 'target_object_pose'

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

    def __init__(self, node, scene_service=PLANNING_SCENE, target_topic=TARGET_TOPIC):
        self.node = node
        self.scene = node.create_client(GetPlanningScene, scene_service)
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
