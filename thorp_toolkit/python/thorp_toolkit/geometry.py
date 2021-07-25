from math import *
from copy import deepcopy
from numbers import Number
from singleton import Singleton

import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs


def __get_naked_pose(pose):
    """ Return input pose without header and covariance """
    if isinstance(pose, geometry_msgs.PoseWithCovarianceStamped):
        return pose.pose.pose
    elif isinstance(pose, geometry_msgs.PoseWithCovariance):
        return pose.pose
    elif isinstance(pose, geometry_msgs.PoseStamped):
        return pose.pose
    elif isinstance(pose, geometry_msgs.Pose):
        return pose
    else:
        raise rospy.ROSException("Input parameter is not a geometry_msgs pose!")


def __set_naked_pose(pose, naked_pose):
    """ Return input pose placing position and rotation with those on naked_pose """
    if not isinstance(naked_pose, geometry_msgs.Pose):
        raise rospy.ROSException("Input parameter naked_pose is not a geometry_msgs.Pose!")
    if isinstance(pose, geometry_msgs.PoseWithCovarianceStamped):
        pose.pose.pose = naked_pose
    elif isinstance(pose, geometry_msgs.PoseWithCovariance):
        pose.pose = naked_pose
    elif isinstance(pose, geometry_msgs.PoseStamped):
        pose.pose = naked_pose
    elif isinstance(pose, geometry_msgs.Pose):
        pose = naked_pose
    else:
        raise rospy.ROSException("Input parameter pose is not any of geometry_msgs' poses!")
    return pose


def __get_naked_poses(pose1, pose2):
    """ Return input poses without headers and covariances """
    return __get_naked_pose(pose1), __get_naked_pose(pose2)


def norm_angle(angle):
    """ Normalize an angle between -pi and +pi """
    angle = angle % (2 * pi)
    if angle > pi:
        angle -= 2 * pi
    return angle


def angles_diff(angle1, angle2):
    """ Normalized difference between two angles """
    return norm_angle(angle2 - angle1)


def heading(pose1, pose2=None):
    """ Heading angle from one pose to another.
        Poses are assumed to have the same reference frame. """
    if not pose2:
        pose2 = pose1
        pose1 = geometry_msgs.Pose()  # 0, 0, 0 pose, i.e. origin
    p1, p2 = __get_naked_poses(pose1, pose2)
    return atan2(p2.position.y - p1.position.y, p2.position.x - p1.position.x)


def distance(x1, y1, x2, y2):
    """ Euclidean distance between 2D points """
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


def distance_2d(pose1, pose2=None):
    """ Euclidean distance between 2D poses; z coordinate is ignored.
        Poses are assumed to have the same reference frame. """
    if not pose2:
        pose2 = pose1
        pose1 = geometry_msgs.Pose()  # 0, 0, 0 pose, i.e. origin
    p1, p2 = __get_naked_poses(pose1, pose2)
    return sqrt(pow(p2.position.x - p1.position.x, 2)
              + pow(p2.position.y - p1.position.y, 2))


def distance_3d(pose1, pose2=None):
    """ Euclidean distance between 3D poses.
        Poses are assumed to have the same reference frame. """
    if not pose2:
        pose2 = pose1
        pose1 = geometry_msgs.Pose()  # 0, 0, 0 pose, i.e. origin
    p1, p2 = __get_naked_poses(pose1, pose2)
    return sqrt(pow(p2.position.x - p1.position.x, 2)
              + pow(p2.position.y - p1.position.y, 2)
              + pow(p2.position.z - p1.position.z, 2))


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


def quaternion_msg_from_yaw(theta):
    """ Create a geometry_msgs/Quaternion from heading """
    return geometry_msgs.Quaternion(*quaternion_from_euler(0, 0, theta))


def quaternion_msg_from_rpy(roll, pitch, yaw):
    """ Create a geometry_msgs/Quaternion from roll, pitch, yaw """
    return geometry_msgs.Quaternion(*quaternion_from_euler(roll, pitch, yaw))


def normalize_quaternion(q):
    """ Normalize quaternion """
    norm = q.x**2 + q.y**2 + q.z**2 + q.w**2
    s = norm**(-0.5)
    q.x *= s
    q.y *= s
    q.z *= s
    q.w *= s


def create_3d_point(x, y, z, frame=None):
    """ Create a geometry_msgs/Point or geometry_msgs/PointStamped
        (if frame is provided) from 3D coordinates """
    point = geometry_msgs.PointStamped()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    if frame:
        point.header.frame_id = frame
        return point
    else:
        return point.point


def create_2d_pose(x, y, theta, frame=None):
    """ Create a geometry_msgs/Pose or geometry_msgs/PoseStamped
        (if frame is provided) from 2D coordinates and heading """
    pose = geometry_msgs.PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = quaternion_msg_from_yaw(theta)
    if frame:
        pose.header.frame_id = frame
        return pose
    else:
        return pose.pose


def create_3d_pose(x, y, z, roll, pitch, yaw, frame=None):
    """ Create a geometry_msgs/Pose or geometry_msgs/PoseStamped
        (if frame is provided) from 3D coordinates and Euler angles """
    pose = geometry_msgs.PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation = quaternion_msg_from_rpy(roll, pitch, yaw)
    if frame:
        pose.header.frame_id = frame
        return pose
    else:
        return pose.pose


def get_pose_from_co(co, stamped=False):
    """ Get the pose for a moveit_msgs/CollisionObject. We try first meshes, then primitives and finally planes """
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


def get_pose_from_aco(aco, stamped=False):
    """ Get the pose for a moveit_msgs/AttachedCollisionObject """
    return get_pose_from_co(aco.object, stamped)


def get_size_from_co(co):
    """ Get the size for a moveit_msgs/CollisionObject. We try first meshes, then primitives """
    if len(co.meshes):
        mesh = co.meshes[0]
        if mesh.vertices < 2:
            return [0, 0, 0]
        vmin = [float('+Inf')] * 3
        vmax = [float('-Inf')] * 3
        for pt in mesh.vertices:
            vpt = [pt.x, pt.y, pt.z]
            for i in range(3):
                vmin[i] = min(vmin[i], vpt[i])
                vmax[i] = max(vmax[i], vpt[i])
        return [vmax[0] - vmin[0], vmax[1] - vmin[1], vmax[2] - vmin[2]]
    if len(co.primitives):
        return co.primitives[0].dimensions

    raise Exception("Collision object contain no meshes nor primitives")


def to_pose2d(pose):
    if isinstance(pose, geometry_msgs.PoseStamped):
        p = pose.pose
    elif isinstance(pose, geometry_msgs.Pose):
        p = pose
    else:
        raise rospy.ROSException("Input parameter pose is not a valid geometry_msgs pose object")
    return geometry_msgs.Pose2D(p.position.x, p.position.y, yaw(p))


def to_pose3d(pose, timestamp=rospy.Time(), frame=None):
    if isinstance(pose, geometry_msgs.Pose2D):
        p = geometry_msgs.Pose(geometry_msgs.Point(pose.x, pose.y, 0.0), quaternion_msg_from_yaw(pose.theta))
        if not frame:
            return p
        return geometry_msgs.PoseStamped(std_msgs.Header(0, timestamp, frame), p)
    raise rospy.ROSException("Input parameter pose is not a geometry_msgs.Pose2D object")


def to_transform(pose, child_frame=None):
    if isinstance(pose, geometry_msgs.Pose2D):
        return geometry_msgs.Transform(geometry_msgs.Vector3(pose.x, pose.y, 0.0), quaternion_msg_from_yaw(pose.theta))
    elif isinstance(pose, geometry_msgs.Pose):
        return geometry_msgs.Transform(geometry_msgs.Vector3(pose.position.x, pose.position.y, pose.position.z),
                                       pose.orientation)
    elif isinstance(pose, geometry_msgs.PoseStamped):
        p = pose.pose
        tf = geometry_msgs.Transform(geometry_msgs.Vector3(p.position.x, p.position.y, p.position.z), p.orientation)
        return geometry_msgs.TransformStamped(pose.header, child_frame, tf)

    raise rospy.ROSException("Input parameter pose is not a valid geometry_msgs pose object")


def point2d2str(point):
    """ Provide a string representation of a geometry_msgs 2D point """
    if isinstance(point, geometry_msgs.Point):
        p = point
        f = ''
    elif isinstance(point, geometry_msgs.PointStamped):
        p = point.point
        f = ', ' + point.header.frame_id
    else:
        raise rospy.ROSException("Input parameter point is not a valid geometry_msgs point object")
    return "[x: {:.2f}, y: {:.2f}{}]".format(p.x, p.y, f)


def point3d2str(point):
    """ Provide a string representation of a geometry_msgs 3D point """
    if isinstance(point, geometry_msgs.Point):
        p = point
        f = ''
    elif isinstance(point, geometry_msgs.PointStamped):
        p = point.point
        f = ', ' + point.header.frame_id
    else:
        raise rospy.ROSException("Input parameter point is not a valid geometry_msgs point object")
    return "[x: {:.2f}, y: {:.2f}, z: {:.2f}{}]".format(p.x, p.y, p.z, f)


def pose2d2str(pose):
    """ Provide a string representation of a geometry_msgs 2D pose """
    if isinstance(pose, geometry_msgs.Pose):
        p = pose
        f = ''
    elif isinstance(pose, geometry_msgs.PoseStamped):
        p = pose.pose
        f = ', ' + pose.header.frame_id
    else:
        raise rospy.ROSException("Input parameter pose is not a valid geometry_msgs pose object")
    return "[x: {:.2f}, y: {:.2f}, yaw: {:.2f}{}]".format(p.position.x, p.position.y, yaw(p), f)


def pose3d2str(pose):
    """ Provide a string representation of a geometry_msgs 3D pose """
    if isinstance(pose, geometry_msgs.Pose):
        p = pose
        f = ''
    elif isinstance(pose, geometry_msgs.PoseStamped):
        p = pose.pose
        f = ', ' + pose.header.frame_id
    else:
        raise rospy.ROSException("Input parameter pose is not a valid geometry_msgs pose object")
    return "[x: {:.2f}, y: {:.2f}, z: {:.2f}, roll: {:.2f}, pitch: {:.2f}, yaw: {:.2f}{}]" \
           .format(p.position.x, p.position.y, p.position.z, roll(p), pitch(p), yaw(p), f)


def translate_pose(pose, delta, axis_or_theta, relative=True):
    """ Apply a displacement to a geometry_msgs pose along a given angle or axis (x, y or z).
        If relative is false, the translation ignores pose's orientation """
    p = __get_naked_pose(pose)

    if isinstance(axis_or_theta, Number):
        theta = axis_or_theta
    elif axis_or_theta == 'x':
        theta = 0.0
    elif axis_or_theta == 'y':
        theta = pi/2
    elif axis_or_theta == 'z':
        p.position.z += delta
        return __set_naked_pose(pose, p)
    else:
        raise rospy.ROSException(axis_or_theta + " is neither a number nor a valid axis ('x', 'y' or 'z')")
    if relative:
        theta = norm_angle(theta + yaw(p))
    p.position.x += cos(theta) * delta
    p.position.y += sin(theta) * delta
    return __set_naked_pose(pose, p)


def rotate_pose(pose, theta, euler):
    """ Rotate a geometry_msgs pose along the given euler angle (roll, pitch or yaw) """
    p = __get_naked_pose(pose)
    if euler == 'roll':
        new_roll = norm_angle(roll(p) + theta)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quaternion_from_euler(new_roll, 0, 0)
    elif euler == 'pitch':
        new_pitch = norm_angle(pitch(p) + theta)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quaternion_from_euler(0, new_pitch, 0)
    elif euler == 'yaw':
        new_yaw = norm_angle(yaw(p) + theta)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quaternion_from_euler(0, 0, new_yaw)
    else:
        raise rospy.ROSException(euler + " is not a valid euler angle (roll, pitch or yaw)")
    return pose


def transform_pose(pose, tf):
    """ Transform the given pose with the given transform """
    # do_transform_pose expects a stamped pose, but it ignores the header
    if isinstance(pose, geometry_msgs.Pose2D):
        p = to_pose3d(pose, frame='dummy')  # just force to_pose3d return a stamped pose
    elif isinstance(pose, geometry_msgs.Pose):
        p = geometry_msgs.PoseStamped(None, pose)
    elif isinstance(pose, geometry_msgs.PoseStamped):
        p = pose
    else:
        raise rospy.ROSException("Input parameter pose is not a valid geometry_msgs pose object")

    return tf2_geometry_msgs.do_transform_pose(p, tf)


def apply_transform(pose, tf):
    """ Apply the given transform to a stamped pose, keeping its reference frame """
    if not isinstance(pose, geometry_msgs.PoseStamped):
        raise rospy.ROSException("Input parameter pose is not a valid geometry_msgs stamped pose")

    p = deepcopy(pose)
    p = tf2_geometry_msgs.do_transform_pose(p, tf)
    p.header = pose.header
    return p


def same_pose(pose1, pose2, xy_tolerance=0.0001, yaw_tolerance=0.0001):
    """
    Compares two poses to be (nearly) the same within tolerance margins, ignoring their frame.
    @param pose1 first pose
    @param pose2 second pose
    @param xy_tolerance linear distance tolerance
    @param yaw_tolerance angular distance tolerance
    @return true if both poses are the same within tolerance margins
    """
    return distance_3d(pose1, pose2) <= xy_tolerance and abs(angles_diff(yaw(pose1), yaw(pose2))) <= yaw_tolerance


class TF2:
    __metaclass__ = Singleton

    def __init__(self):
        """ Singleton encapsulating a tf2 listener and a broadcaster """
        try:
            self.__buff__ = tf2_ros.Buffer()
            self.__list__ = tf2_ros.TransformListener(self.__buff__)
            self.__stbc__ = tf2_ros.StaticTransformBroadcaster()
        except rospy.ROSException as err:
            rospy.logerr("Could not start tf buffer client: " + str(err))
            raise err

    def transform_pose(self, pose_in, frame_from, frame_to, timeout=rospy.Duration(2.0)):
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
            return self.__buff__.lookup_transform(frame_from, frame_to, timestamp, timeout)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                rospy.exceptions.ROSInterruptException) as err:
            raise rospy.ROSException("Could not lookup transform from %s to %s: %s" % (frame_from, frame_to, str(err)))

    def publish_transform(self, transform):
        self.__stbc__.sendTransform(transform)
