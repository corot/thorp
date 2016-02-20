#!/usr/bin/env python

from math import *

import tf
import rospy

import geometry_msgs.msg as geometry_msgs


def heading(pose1, pose2=None):
    if not pose2:
        pose2 = pose1
        pose1 = geometry_msgs.Pose() # 0, 0, 0 pose, i.e. origin
    return atan2(pose2.position.y - pose1.position.y, pose2.position.x - pose1.position.x)

def distance_2d(pose1, pose2=None):
    if not pose2:
        pose2 = pose1
        pose1 = geometry_msgs.Pose() # 0, 0, 0 pose, i.e. origin
    return sqrt(pow(pose2.position.x - pose1.position.x, 2)
              + pow(pose2.position.y - pose1.position.y, 2))
    
def distance_3d(pose1, pose2=None):
    if not pose2:
        pose2 = pose1
        pose1 = geometry_msgs.Pose() # 0, 0, 0 pose, i.e. origin
    return sqrt(pow(pose2.position.x - pose1.position.x, 2)
              + pow(pose2.position.y - pose1.position.y, 2)
              + pow(pose2.position.z - pose1.position.z, 2))
    
def get_pose_from_co(co, stamped=False):
    ''' Get the pose for a moveit_msgs/CollisionObject. We try first meshes, then primitives and finally planes.
        TODO: make another function to calculate the centroid OF ALL THREE ELEMENTS and provide it!
        More TODO: put this in thorp_toolkit, and also a C++ version and aco '''
    if len(co.mesh_poses):
        pose = co.mesh_poses[0]
    elif len(co.primitive_poses):
        pose = co.primitive_poses[0]
    elif len(co.plane_poses):
        pose = co.plane_poses[0]
    else:  
        raise Exception("Collision object contain no poses")

    if not stamped:
        return pose
    pose_stamped = geometry_msgs.PoseStamped()
    pose_stamped.header = co.header
    pose_stamped.pose = pose
    return pose_stamped
    
def transform_pose(target_frame, pose_stamped):
    transform_pose.tf_listener = tf.TransformListener()
    transform_pose.tf_listener.waitForTransform(target_frame, pose_stamped.header.frame_id,
                                                rospy.Time(), rospy.Duration(30.0))
    return transform_pose.tf_listener.transformPose(target_frame, pose_stamped)
#     if tf_listener.frameExists("/base_link") and tf_listener.frameExists("/map"):
#         t = tf_listener.getLatestCommonTime("/base_link", "/map")
#         position, quaternion = tf_listener.lookupTransform("/base_link", "/map", t)