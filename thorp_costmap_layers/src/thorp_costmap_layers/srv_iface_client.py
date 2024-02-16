import rospy

from thorp_costmap_layers.msg import Object
from thorp_costmap_layers.srv import UpdateObjects, UpdateObjectsResponse
from thorp_toolkit.geometry import pose2d2str
from thorp_toolkit.singleton import Singleton


class SemanticLayer(metaclass=Singleton):
    """ Singleton providing a simplified interface for adding/removing objects to the costmaps semantic layer """

    def __init__(self):
        self._lcm_sl_srv = rospy.ServiceProxy('move_base_flex/local_costmap/semantic_layer/update_objects',
                                              UpdateObjects)
        self._gcm_sl_srv = rospy.ServiceProxy('move_base_flex/global_costmap/semantic_layer/update_objects',
                                              UpdateObjects)
        self._lcm_sl_srv.wait_for_service(30.0)
        self._gcm_sl_srv.wait_for_service(30.0)

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
                resp = self._lcm_sl_srv([obj])
                if resp.code != UpdateObjectsResponse.SUCCESS:
                    rospy.logerr("Unable to add object %s to %s costmap: %d", name, costmap, resp.code)
                    return False
            if costmap in ['global', 'both']:
                resp = self._gcm_sl_srv([obj])
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
                resp = self._lcm_sl_srv([obj])
                if resp.code != UpdateObjectsResponse.SUCCESS:
                    rospy.logerr("Unable to remove object %s from %s costmap: %d", name, costmap, resp.code)
                    return False
            if costmap in ['global', 'both']:
                resp = self._gcm_sl_srv([obj])
                if resp.code != UpdateObjectsResponse.SUCCESS:
                    rospy.logerr("Unable to remove object %s from %s costmap: %d", name, costmap, resp.code)
                    return False
            rospy.loginfo("Removed object %s of type %s from %s costmap", name, type, costmap)
            return True
        except rospy.ServiceException as err:
            rospy.logerr("Unable to remove object %s from %s costmap: %s", name, costmap, str(err))
            return False
