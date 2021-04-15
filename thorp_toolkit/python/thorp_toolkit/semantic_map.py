from copy import deepcopy

from singleton import Singleton
from spatial_hash import SpatialHash, Rect
from visualization import Visualization


class SemanticMap:
    """
    Singleton providing an ultra-simple semantic map.
    By now just used to recognize visited tables.
    """
    __metaclass__ = Singleton

    def __init__(self):
        self._hashmap = SpatialHash(0.5)
        self._visual = Visualization()
        self._colors = {'table': [0.6, 0.3, 0.0, 0.4],  # wood brown
                        'wall': [0.8, 0.25, 0.3, 0.4]}  # red brick

    def add_object(self, obj, type, pose, size):
        """
        Add an object to the semantic map.
        :param obj: object
        :param type: object type (table, wall, etc.)
        :param pose: object pose (PoseStamped)
        :param size: object size (2 or 3 floats list)
        """
        if len(size) == 2:
            size += 0.001,
        width, length, height = size
        name = str(obj)
        name_pose = deepcopy(pose)
        name_pose.pose.position.z += 0.2
        self._hashmap.add_rect(Rect.from_cwh(pose.pose.position, width, length), obj)
        self._visual.add_box_marker(pose, size, self._colors.get(type, [0.5] * 3))
        self._visual.add_text_marker(name_pose, name, 0.2, self._colors.get(type, [0.5] * 3))
        self._visual.publish_markers(5000)

    def remove_object(self, obj):
        """
        Remove an object from the semantic map.
        :param obj: object
        """
        self._hashmap.remove(obj)
        # TODO: remove visual marker (probably I need to clear all markers with create_geometry_primitive_marker and re-add non-removed ones)
        # with that I can have a semantic_map ns, so no need for the 5000 on publish_markers!

    def objects_at(self, pose, size):
        """
        Retrieve objects within an area.
        :param pose: area pose (PoseStamped)
        :param size: area size (2 floats list)
        """
        return self._hashmap.query(Rect.from_cwh(pose.pose.position, *size))
        # TODO: can check something else??? like name

    def objects(self, type=None):
        """
        Get all objects of a given type.
        :param type: object type (table, wall, etc.)
        """
        return self._hashmap.content()

    def objects_count(self, type=None):
        """
        Count objects of a given type.
        :param type: object type (table, wall, etc.)
        """
        # TODO how to filter by type???? objs = self._hashmap.content()
        return self._hashmap.count()
