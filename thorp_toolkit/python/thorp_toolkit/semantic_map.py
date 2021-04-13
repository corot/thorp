from singleton import Singleton
from spatial_hash import SpatialHash, Rect


class SemanticMap:
    """
    Singleton providing an ultra-simple semantic map.
    By now just used to recognize visited tables.
    """
    __metaclass__ = Singleton

    def __init__(self):
        self._hashmap = SpatialHash(0.5)

    def add_object(self, obj, type, pose, size):
        """
        Add an object to the semantic map.
        :param obj: object
        :param type: object type (table, wall, etc.)
        :param pose: object pose (PoseStamped)
        :param size: object size (2 floats list)
        """
        self._hashmap.add_rect(Rect.from_cwh(pose.pose.position, *size), obj)

    def remove_object(self, obj):
        """
        Remove an object from the semantic map.
        :param obj: object
        """
        pass  #TODO doesn't make sense to use self._hashmap.remove_rect(),,,  it requires a rect!!!

    def objects_at(self, pose, size):
        """
        Retrieve objects within an area.
        :param pose: area pose (PoseStamped)
        :param size: area size (2 floats list)
        """
        return self._hashmap.query(Rect.from_cwh(pose.pose.position, *size))
        # TODO: can check something else??? like name

    def objects_count(self, type=None):
        """
        Count objects of a given type.
        :param type: object type (table, wall, etc.)
        """
        # TODO how to filter by type???? objs = self._hashmap.content()
        return self._hashmap.count()
