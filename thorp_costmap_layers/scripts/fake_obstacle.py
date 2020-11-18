#! /usr/bin/env python

"""
Created on May 17, 2017

@author: santossimon
"""

import json
import rospy

from thorp_toolkit.geometry import TF2
from moveit_msgs.msg import PlanningSceneWorld, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

def __add_virtual_obstacle(name, x, y, operation=CollisionObject.ADD):
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.BOX
    sp.dimensions = [0.5, 0.5, 0.1]
    spp = PoseStamped()
    spp.header.frame_id = 'map'
    spp.pose.position.x = x
    spp.pose.position.y = y
    spp.pose.position.z = 0.05
    spp.pose.orientation.w = 1
    co = CollisionObject()
    co.operation = operation
    co.id = name
    co.type.db = json.dumps({'table': 'NONE', 'type': 'obstacle', 'name': co.id})
    co.header.frame_id = 'map'
    co.header.stamp = rospy.get_rostime()
    co.primitives.append(sp)
    co.primitive_poses.append(TF2().transform_pose(spp, spp.header.frame_id, co.header.frame_id).pose)
    psw = PlanningSceneWorld()
    psw.collision_objects.append(co)
    obstacle_pub.publish(psw)
    rospy.loginfo("Added a fake obstacle named '%s'", name)

def __remove_virtual_obstacle(name):
    co = CollisionObject()
    co.operation = CollisionObject.REMOVE
    co.id = name
    co.type.db = json.dumps({'table': 'NONE', 'type': 'obstacle', 'name': co.id})
    psw = PlanningSceneWorld()
    psw.collision_objects.append(co)
    obstacle_pub.publish(psw)
    rospy.loginfo("Removed a fake obstacle named '%s'", name)


rospy.init_node('fake_obstacle')

# Publish fake collision objects to the planning scene world; add and remove
obstacle_pub = rospy.Publisher('move_base_flex/local_costmap/semantic_layer/add_objects',
                               PlanningSceneWorld, queue_size=1)

rospy.sleep(2)
__add_virtual_obstacle('KK1', 2, 1.0)
rospy.sleep(5)
__add_virtual_obstacle('KK2', 2.8, 3.0)
rospy.sleep(5)
__add_virtual_obstacle('KK2', 0.6, 0.2)
rospy.sleep(5)
__add_virtual_obstacle('KK2', 1.0, 0.4)
rospy.sleep(5)
__add_virtual_obstacle('KK2', 1.8, 0.4, CollisionObject.MOVE)
rospy.sleep(5)
__add_virtual_obstacle('KK2', 2.2, 0.6, CollisionObject.MOVE)
rospy.sleep(5)
__add_virtual_obstacle('KK2', 2.6, 0.8, CollisionObject.MOVE)
rospy.sleep(5)

__remove_virtual_obstacle('KK1')
rospy.sleep(5)
__remove_virtual_obstacle('KK2')
rospy.sleep(5)