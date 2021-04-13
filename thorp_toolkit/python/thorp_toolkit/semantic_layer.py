import json

import rospy

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from thorp_msgs.msg import ThorpError
from thorp_msgs.srv import UpdateCollisionObjs

from geometry import pose2d2str, TF2
from singleton import Singleton


class SemanticLayer:
    """ Singleton providing a simplified interface for adding/removing objects to the costmaps semantic layer """
    __metaclass__ = Singleton

    def __init__(self):
        self._lcm_sl_srv = rospy.ServiceProxy('move_base_flex/local_costmap/semantic_layer/update_objects',
                                              UpdateCollisionObjs)
        self._gcm_sl_srv = rospy.ServiceProxy('move_base_flex/global_costmap/semantic_layer/update_objects',
                                              UpdateCollisionObjs)
        self._lcm_sl_srv.wait_for_service(30.0)
        self._gcm_sl_srv.wait_for_service(30.0)

    def add_object(self, name, type, pose, size, costmap='both'):
        """
        Add an semantic object to one or both costmaps. Pose will be transformed to map frame.
        :param name: object name
        :param type: object type (obstacle, free space, etc.)
        :param pose: object pose (PoseStamped)
        :param size: object size (3 floats list)
        :param costmap: 'local', 'global' or 'both'
        """
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = size
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.type.db = json.dumps({'table': 'NONE', 'type': type, 'name': name})
        co.header.frame_id = 'map'
        co.header.stamp = pose.header.stamp
        co.primitives.append(sp)
        co.primitive_poses.append(TF2().transform_pose(pose, pose.header.frame_id, co.header.frame_id).pose)
        try:
            if costmap in ['local', 'both']:
                resp = self._lcm_sl_srv([co])
                if resp.error.code != ThorpError.SUCCESS:
                    rospy.logerr("Unable to add object %s to %s costmap: %d", name, costmap, resp.error.code)
                    return False
            if costmap in ['global', 'both']:
                resp = self._gcm_sl_srv([co])
                if resp.error.code != ThorpError.SUCCESS:
                    rospy.logerr("Unable to add object %s to %s costmap: %d", name, costmap, resp.error.code)
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
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.id = name
        co.type.db = json.dumps({'table': 'NONE', 'type': type, 'name': name})
        try:
            if costmap in ['local', 'both']:
                resp = self._lcm_sl_srv([co])
                if resp.error.code != ThorpError.SUCCESS:
                    rospy.logerr("Unable to remove object %s from %s costmap: %d", name, costmap, resp.error.code)
                    return False
            if costmap in ['global', 'both']:
                resp = self._gcm_sl_srv([co])
                if resp.error.code != ThorpError.SUCCESS:
                    rospy.logerr("Unable to remove object %s from %s costmap: %d", name, costmap, resp.error.code)
                    return False
            rospy.loginfo("Removed object %s of type %s from %s costmap", name, type, costmap)
            return True
        except rospy.ServiceException as err:
            rospy.logerr("Unable to remove object %s from %s costmap: %s", name, costmap, str(err))
            return False
