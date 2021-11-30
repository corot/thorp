import rospy

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_commander.planning_scene_interface import PlanningSceneInterface

from .singleton import Singleton


class PlanningScene(metaclass=Singleton):
    """ Singleton providing a simplified interface to the planning scene """
    def __init__(self):
        self._objs_on_tray = set()
        self.__psi__ = PlanningSceneInterface(synchronous=True)
        rospy.sleep(0.25)

    def add_tray(self, pose, size):
        self.__psi__.add_box('tray', pose, size)

    def remove_all(self, keep_objs_on_tray=False):
        """
        Remove all objects from planning scene, optionally sparing those on the tray and the tray itself
        """
        if keep_objs_on_tray:
            objs_to_remove = set(self.__psi__.get_known_object_names()) - self._objs_on_tray - {'tray'}
            for obj in objs_to_remove:
                self.__psi__.remove_world_object(obj)
        else:
            self.__psi__.remove_world_object()

    def remove_obj(self, obj_name=None):
        """
        Remove an object from planning scene, or all if no name is provided
        """
        self.__psi__.remove_world_object(obj_name)

    def displace_obj(self, obj_name, new_pose):
        co = self.get_obj(obj_name)
        co.header = new_pose.header
        if co.primitive_poses:
            co.primitives = []
            co.primitive_poses[0] = new_pose.pose
        if co.mesh_poses:
            co.meshes = []
            co.mesh_poses[0] = new_pose.pose
        if co.plane_poses:
            co.planes = []
            co.plane_poses[0] = new_pose.pose
        co.operation = CollisionObject.MOVE
        self.__psi__._pub_co.publish(co)

    def move_obj_to_tray(self, obj_name, pose_on_tray):
        self.displace_obj(obj_name, pose_on_tray)
        self._objs_on_tray.add(obj_name)
        return

    def get_obj(self, obj_name):
        objs = self.__psi__.get_objects([obj_name])
        if not objs:
            rospy.logerr("Object '%s' not found on planning scene")
            return None
        assert len(objs) == 1, "%d objects with the same name???" % len(objs)
        return objs[obj_name]
