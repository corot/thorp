import rospy

from moveit_msgs.msg import CollisionObject
from moveit_commander.planning_scene_interface import PlanningSceneInterface

from singleton import Singleton


class PlanningScene:
    """ Singleton providing a simplified interface to the planning scene """
    __metaclass__ = Singleton

    def __init__(self):
        self.__psi__ = PlanningSceneInterface(synchronous=True)
        rospy.sleep(0.25)

    def add_tray(self, pose, size):
        self.__psi__.add_box('tray', pose, size)

    def remove_obj(self, obj_name=None):
        """
        Remove an object from planning scene, or all if no name is provided
        """
        self.__psi__.remove_world_object(obj_name)

    def displace_obj(self, obj_name, new_pose):
        objs = self.__psi__.get_objects([obj_name])
        if not objs:
            rospy.logerr("Object '%s' not found on planning scene")
            return False
        for co in objs.values():
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
