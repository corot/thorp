import json

import rospy

from moveit_msgs.msg import CollisionObject, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive

from geometry import pose2d2str, TF2
from singleton import Singleton


class SemanticMap:
    """ Singleton providing a simplified interface for adding/removing objects to the semantic map """
    __metaclass__ = Singleton

    def __init__(self):
        self._lcm_sl_pub = rospy.Publisher('move_base_flex/local_costmap/semantic_layer/add_objects',
                                           PlanningSceneWorld, queue_size=1)
        self._gcm_sl_pub = rospy.Publisher('move_base_flex/global_costmap/semantic_layer/add_objects',
                                           PlanningSceneWorld, queue_size=1)
        rospy.sleep(0.25)  # wait a moment until the publishers are ready

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
        psw = PlanningSceneWorld()
        psw.collision_objects.append(co)
        if costmap in ['local', 'both']:
            self._lcm_sl_pub.publish(psw)
        if costmap in ['global', 'both']:
            self._gcm_sl_pub.publish(psw)
        rospy.loginfo("Added object %s of type %s at %s to %s costmap", name, type, pose2d2str(pose), costmap)

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
        psw = PlanningSceneWorld()
        psw.collision_objects.append(co)
        if costmap in ['local', 'both']:
            self._lcm_sl_pub.publish(psw)
        if costmap in ['global', 'both']:
            self._gcm_sl_pub.publish(psw)
        rospy.loginfo("Removed object %s of type %s from %s costmap", name, type, costmap)
