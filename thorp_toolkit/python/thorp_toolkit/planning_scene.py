import rospy

from moveit_commander.planning_scene_interface import PlanningSceneInterface

from .singleton import Singleton


class PlanningScene:
    """ Singleton providing a simplified interface to the planning scene """
    __metaclass__ = Singleton

    def __init__(self, topic='~visual_markers', lifetime=60):
        self.__psi__ = PlanningSceneInterface()
        rospy.sleep(0.5)  # wait a moment until the publisher is ready

    def add_tray(self, pose, size):
        self.__psi__.add_box('tray', pose, size)

    def remove_obj(self, obj_name):
        self.__psi__.remove_world_object(obj_name)
