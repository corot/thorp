import rospy

from thorp_costmap_layers.msg import Object
from thorp_costmap_layers.srv import UpdateObjects, UpdateObjectsResponse, QueryObjects
from thorp_toolkit.geometry import pose2d2str
from thorp_toolkit.singleton import Singleton


class SemanticLayer(metaclass=Singleton):
    """
    Singleton providing a simplified interface for adding/removing/querying objects to/from the costmaps' semantic layer
    """

    def __init__(self):
        self._lcm_sl_update_srv = rospy.ServiceProxy('move_base_flex/local_costmap/semantic_layer/update_objects',
                                                     UpdateObjects)
        self._gcm_sl_update_srv = rospy.ServiceProxy('move_base_flex/global_costmap/semantic_layer/update_objects',
                                                     UpdateObjects)
        self._lcm_sl_query_srv = rospy.ServiceProxy('move_base_flex/local_costmap/semantic_layer/query_objects',
                                                    QueryObjects)
        self._gcm_sl_query_srv = rospy.ServiceProxy('move_base_flex/global_costmap/semantic_layer/query_objects',
                                                    QueryObjects)

        self._lcm_sl_update_srv.wait_for_service(30.0)
        self._gcm_sl_update_srv.wait_for_service(30.0)
        self._lcm_sl_query_srv.wait_for_service(30.0)
        self._gcm_sl_query_srv.wait_for_service(30.0)

    def add_object(self, name, type, pose, size, costmap='both'):
        """
        Add a semantic object to one or both costmaps. Pose will be transformed to map frame.
        :param name: object name
        :param type: object type (obstacle, free space, etc.)
        :param pose: object pose (PoseStamped)
        :param size: object size (Vector3)
        :param costmap: 'local', 'global' or 'both'
        """
        obj = Object()
        obj.operation = Object.ADD
        obj.name = name
        obj.type = type
        obj.pose = pose
        obj.dimensions = size
        try:
            if costmap in ['local', 'both']:
                resp = self._lcm_sl_update_srv([obj])
                if resp.code != UpdateObjectsResponse.SUCCESS:
                    rospy.logerr("Unable to add object %s to %s costmap: %d", name, costmap, resp.code)
                    return False
            if costmap in ['global', 'both']:
                resp = self._gcm_sl_update_srv([obj])
                if resp.code != UpdateObjectsResponse.SUCCESS:
                    rospy.logerr("Unable to add object %s to %s costmap: %d", name, costmap, resp.code)
                    return False
            rospy.loginfo("Added object %s of type %s at %s to %s costmap", name, type, pose2d2str(pose), costmap)
            return True
        except rospy.ServiceException as err:
            rospy.logerr("Unable to add object %s to %s costmap: %s", name, costmap, str(err))
            return False

    def remove_object(self, name, type, costmap='both'):
        """
        Remove an object from one or both costmaps.
        :param name: object name
        :param type: object type
        :param costmap: 'local', 'global' or 'both'
        """
        obj = Object()
        obj.operation = Object.REMOVE
        obj.name = name
        obj.type = type
        try:
            if costmap in ['local', 'both']:
                resp = self._lcm_sl_update_srv([obj])
                if resp.code != UpdateObjectsResponse.SUCCESS:
                    rospy.logerr("Unable to remove object %s from %s costmap: %d", name, costmap, resp.code)
                    return False
            if costmap in ['global', 'both']:
                resp = self._gcm_sl_update_srv([obj])
                if resp.code != UpdateObjectsResponse.SUCCESS:
                    rospy.logerr("Unable to remove object %s from %s costmap: %d", name, costmap, resp.code)
                    return False
            rospy.loginfo("Removed object %s of type %s from %s costmap", name, type, costmap)
            return True
        except rospy.ServiceException as err:
            rospy.logerr("Unable to remove object %s from %s costmap: %s", name, costmap, str(err))
            return False

    def query_objects(self, lower_left, upper_right, costmap='both'):
        """
        Query objects within a bounding box from one or both costmaps.
        :param lower_left: lower left corner of the bounding box (Point)
        :param upper_right: upper right corner of the bounding box (Point)
        :param costmap: 'local', 'global' or 'both'
        :return: list of objects within the bounding box
        """
        objects = []
        try:
            if costmap in ['local', 'both']:
                resp = self._lcm_sl_query_srv(lower_left, upper_right)
                if not resp.objects:
                    rospy.logerr("Query to %s costmap returned no objects", costmap)
                    return []
                objects.extend(resp.objects)
            if costmap in ['global', 'both']:
                resp = self._gcm_sl_query_srv(lower_left, upper_right)
                if not resp.objects:
                    rospy.logerr("Query to %s costmap returned no objects", costmap)
                    return []
                objects.extend(resp.objects)
            return objects
        except rospy.ServiceException as err:
            rospy.logerr("Unable to query objects from %s costmap: %s", costmap, str(err))
            return []

