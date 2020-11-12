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

    def add_obstacle(self, name, pose, size, costmap='both'):
        """
        Add an obstacle to one or both costmaps. Pose will be transformed to map frame.
        :param name: obstacle name
        :param pose: obstacle pose (PoseStamped)
        :param size: obstacle size (3 floats list)
        :param costmap: 'local', 'global' or 'both'
        """
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = size
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.type.db = json.dumps({'table': 'NONE', 'type': 'obstacle', 'name': name})
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
        rospy.loginfo("Added obstacle '%s' at %s to %s costmap", name, pose2d2str(pose), costmap)

    def remove_obstacle(self, name, costmap='both'):
        """
        Remove an obstacle from one or both costmaps.
        :param name: obstacle name
        :param costmap: 'local', 'global' or 'both'
        """
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.id = name
        co.type.db = json.dumps({'table': 'NONE', 'type': 'obstacle', 'name': name})
        psw = PlanningSceneWorld()
        psw.collision_objects.append(co)
        if costmap in ['local', 'both']:
            self._lcm_sl_pub.publish(psw)
        if costmap in ['global', 'both']:
            self._gcm_sl_pub.publish(psw)
        rospy.loginfo("Removed obstacle '%s' from %s costmap", name, costmap)
