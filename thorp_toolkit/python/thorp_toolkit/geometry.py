#!/usr/bin/env python

from math import *
from .singleton import Singleton

import rospy
import tf2_ros
# importing tf2_geometry_msgs to register geometry_msgs types with tf2_ros.TransformRegistration
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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


def get_euler(pose_or_quat):
    """ Get Euler angles from a geometry_msgs pose or quaternion """
    if isinstance(pose_or_quat, geometry_msgs.Quaternion):
        q = pose_or_quat
    elif isinstance(pose_or_quat, geometry_msgs.TransformStamped):
        q = pose_or_quat.transform.rotation
    elif isinstance(pose_or_quat, geometry_msgs.Transform):
        q = pose_or_quat.rotation
    elif isinstance(pose_or_quat, geometry_msgs.PoseStamped):
        q = pose_or_quat.pose.orientation
    elif isinstance(pose_or_quat, geometry_msgs.Pose):
        q = pose_or_quat.orientation
    else:
        raise rospy.ROSException("Input parameter pose_or_quat is not a valid geometry_msgs object")

    return euler_from_quaternion((q.x, q.y, q.z, q.w))


def roll(pose_or_quat):
    """ Get roll from a geometry_msgs pose or quaternion """
    return get_euler(pose_or_quat)[0]


def pitch(pose_or_quat):
    """ Get pitch from a geometry_msgs pose or quaternion """
    return get_euler(pose_or_quat)[1]


def yaw(pose_or_quat):
    """ Get yaw from a geometry_msgs pose or quaternion """
    return get_euler(pose_or_quat)[2]


def quaternion_from_yaw(theta):
    """ Create a geometry_msgs/Quaternion from heading """
    return geometry_msgs.Quaternion(*quaternion_from_euler(0, 0, theta))


def normalize_quaternion(q):
    """ Normalize quaternion """
    norm = q.x**2 + q.y**2 + q.z**2 + q.w**2
    s = norm**(-0.5)
    q.x *= s
    q.y *= s
    q.z *= s
    q.w *= s


def create_2d_pose(x, y, theta, frame=None):
    """ Create a geometry_msgs/Pose or geometry_msgs/PoseStamped
        (if frame is provided) from 2D coordinates and heading """
    pose = geometry_msgs.PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = \
        quaternion_from_euler(0, 0, theta)
    if frame:
        pose.header.frame_id = frame
        return pose
    else:
        return pose.pose


def get_pose_from_co(co, stamped=False):
    """ Get the pose for a moveit_msgs/CollisionObject. We try first meshes, then primitives and finally planes.
        TODO: make another function to calculate the centroid OF ALL THREE ELEMENTS and provide it!
        More TODO: put this in thorp_toolkit, and also a C++ version and aco """
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


def pose2d2str(pose):
    """ Provide a string representation of a geometry_msgs 2D pose """
    return "[x: {:.2f}, y: {:.2f}, yaw: {:.2f}]".format(pose.position.x, pose.position.y, yaw(pose))


def pose3d2str(pose):
    """ Provide a string representation of a geometry_msgs 3D pose """
    return "[x: {:.2f}, y: {:.2f}, z: {:.2f}, roll: {:.2f}, pitch: {:.2f}, yaw: {:.2f}]".format(pose.position.x,
                                                                                                pose.position.y,
                                                                                                pose.position.z,
                                                                                                *get_euler(pose))


class TF2:
    __metaclass__ = Singleton

    def __init__(self):
        """ Creates a global TF2 buffer to use on all tf related functions """
        try:
            self.__buff__ = tf2_ros.Buffer()
            self.__list__ = tf2_ros.TransformListener(self.__buff__)
        except rospy.ROSException as err:
            rospy.logerr("Could not start tf buffer client: " + str(err))
            raise err

    def transform_pose(self, pose_in, frame_from, frame_to, timeout=rospy.Duration(1.0)):
        """ Transform pose_in from one frame to another, or create
        the corresponding pose if None is provided on pose_in """
        if not pose_in:
            pose_in = geometry_msgs.PoseStamped()
            pose_in.header.frame_id = frame_from
            pose_in.header.stamp = rospy.Time(0.0)
            pose_in.pose.orientation.w = 1.0
        try:
            return self.__buff__.transform(pose_in, frame_to, timeout)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                rospy.exceptions.ROSInterruptException) as err:
            raise rospy.ROSException("Could not transform pose from %s to %s: %s" % (frame_from, frame_to, str(err)))

    def lookup_transform(self, frame_from, frame_to, timestamp=rospy.Time(0.0), timeout=rospy.Duration(1.0)):
        try:
            return self.__buff__.lookup_transform(frame_to, frame_from, timestamp, timeout)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                rospy.exceptions.ROSInterruptException) as err:
            raise rospy.ROSException("Could not lookup transform from %s to %s: %s" % (frame_from, frame_to, str(err)))


# TODO: deprecated --> remove
import tf
def transform_pose(target_frame, pose_stamped):
    global tf_listener
    if not tf_listener:
        tf_listener = tf.TransformListener()
        rospy.sleep(0.1)  # wait a moment to allow listener to start filling the buffer

    tf_listener.waitForTransform(target_frame, pose_stamped.header.frame_id,
                                 rospy.Time(), rospy.Duration(30.0))
    return tf_listener.transformPose(target_frame, pose_stamped)
