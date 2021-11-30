import rospy
from copy import deepcopy
from collections import namedtuple

from visualization_msgs.msg import Marker, MarkerArray

from .singleton import Singleton
from .spatial_hash import SpatialHash, Rect
from .visualization import Visualization


SemanticObj = namedtuple('SemanticObj', ['obj', 'name', 'type', 'pose', 'size', 'markers'])


class SemanticMap(metaclass=Singleton):
    """
    Singleton providing an ultra-simple semantic map.
    By now just used to recognize visited tables.
    """

    def __init__(self):
        self._hashmap = SpatialHash(0.5)
        self._colors = {'junk': [0.1, 0.1, 0.1, 0.4],   # dark gray
                        'table': [0.6, 0.3, 0.0, 0.4],  # wood brown
                        'wall': [0.8, 0.25, 0.3, 0.4]}  # red brick
        self._content = {}
        self._viz_pub = rospy.Publisher('semantic_map/markers', MarkerArray, queue_size=1)

    def add_object(self, name, type, pose, size, obj=None):
        """
        Add an object to the semantic map.
        :param name: object's name
        :param type: object type (table, wall, etc.)
        :param pose: object pose (PoseStamped)
        :param size: object size (2 or 3 floats list)
        :param obj: the object itself
        """
        if len(size) == 2:
            size += 0.001,
        width, length, height = size
        name_pose = deepcopy(pose)
        name_pose.pose.position.z += 0.2
        self._hashmap.add_rect(Rect.from_cwh(pose.pose.position, width, length), name)
        color = self._colors.get(type, [0.5] * 3)
        markers = [Visualization.create_geometry_primitive_marker(pose, size, color, Marker.CUBE, name + ' shape'),
                   Visualization.create_text_marker(name_pose, name, 0.2, color, name + ' label')]
        self._content[name] = SemanticObj(obj, name, type, pose, size, markers)
        self.publish_markers()

    def remove_object(self, name):
        """
        Remove an object from the semantic map.
        :param name: object's name
        """
        self._hashmap.remove(name)
        try:
            del self._content[name]
            self.publish_markers()
            return True
        except KeyError:
            rospy.logwarn("Attempting to remove object '%s' not found on semantic map", name)
            return False

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

    def publish_markers(self):
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for obj in self._content.values():
            markers.markers.extend(obj.markers)
        self._viz_pub.publish(markers)
