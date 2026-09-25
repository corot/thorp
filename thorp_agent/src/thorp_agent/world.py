"""
What is true of the robot right now, read rather than remembered.

Anything that moves the robot is a behavior tree, declared in capabilities.yaml and run through
bt_server. What is here is the other half: observations, which touch nothing and answer whether
the states that file names actually hold. They ride on every capability result, so knowing where
things stand costs the agent no call of its own.

One state in the glossary has no reading behind it. Nothing reports "the robot is parked against
a table", so at_a_table stays an assumption from whatever attach_to_table last did, and anybody
who pushes the robot invalidates it silently.
"""

import rospy

PLANNING_SCENE = "/get_planning_scene"
GRIPPER_BUSY = ("/manipulation/gripper_busy", "/gripper_busy")
TARGET_TOPIC = "target_object_predicted_pose"

# How recently the tracker must have published for the target to count as still in view. The
# follower gives up after 5 s of silence, so this is the shorter of the two answers.
TARGET_FRESH_FOR = 3.0


class _LastHeard(object):
    """Subscribes once and remembers only when a message last arrived, not what it said."""

    def __init__(self, topic):
        self.at = None
        try:
            rospy.Subscriber(topic, rospy.AnyMsg, self._heard, queue_size=1)
        except Exception as e:
            rospy.logwarn("cannot watch %s: %s", topic, e)

    def _heard(self, _):
        self.at = rospy.Time.now()

    def fresh(self, within):
        return bool(self.at) and (rospy.Time.now() - self.at).to_sec() < within


class World(object):
    """
    Reads the robot's situation on demand.

    Every reading is optional: the agent talks to whatever stack happens to be running, and a
    missing service means that fact is unknown, not that the run failed. An absent key says
    "couldn't look" and a present one says what was seen, which are different answers and the
    agent should be able to tell them apart.
    """

    def __init__(self, scene_service=PLANNING_SCENE, target_topic=TARGET_TOPIC):
        self.scene_service = scene_service
        self._scene = None
        self._gripper = None
        self._target = _LastHeard(target_topic)

    def observe(self):
        seen = {}
        scene = self._planning_scene()
        if scene is not None:
            attached = scene.robot_state.attached_collision_objects
            seen["holding"] = attached[0].object.id if attached else None
            seen["scene_objects"] = sorted(o.id for o in scene.world.collision_objects)
        busy = self._gripper_busy()
        if busy is not None:
            seen["gripper_closed_on_something"] = busy
        seen["target_in_view"] = self._target.fresh(TARGET_FRESH_FOR)
        return seen

    def _planning_scene(self):
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest

        if self._scene is None:
            try:
                rospy.wait_for_service(self.scene_service, timeout=2.0)
            except rospy.ROSException:
                return None
            self._scene = rospy.ServiceProxy(self.scene_service, GetPlanningScene)

        request = GetPlanningSceneRequest()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        try:
            return self._scene(request).scene
        except rospy.ServiceException as e:
            rospy.logwarn("cannot read the planning scene: %s", e)
            self._scene = None
            return None

    def _gripper_busy(self):
        """
        Whether the gripper is physically closed on something, which the planning scene cannot
        say: the two disagreeing is how a dropped object shows up.
        """
        from std_srvs.srv import Trigger

        if self._gripper is None:
            for name in GRIPPER_BUSY:
                try:
                    rospy.wait_for_service(name, timeout=1.0)
                except rospy.ROSException:
                    continue
                self._gripper = rospy.ServiceProxy(name, Trigger)
                break
            else:
                return None
        try:
            return bool(self._gripper().success)
        except rospy.ServiceException as e:
            rospy.logwarn("cannot read the gripper: %s", e)
            self._gripper = None
            return None
