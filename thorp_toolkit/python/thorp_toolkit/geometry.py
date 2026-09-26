import numpy as np
from numpy import pi
from numbers import Number

import time

import rclpy
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from rclpy.time import Time
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

from .common import node
from .singleton import Singleton


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
        raise ValueError("Input parameter is not a geometry_msgs pose!")


def __set_naked_pose(pose, naked_pose):
    """ Return input pose placing position and rotation with those on naked_pose """
    if not isinstance(naked_pose, geometry_msgs.Pose):
        raise ValueError("Input parameter naked_pose is not a geometry_msgs.Pose!")
    if isinstance(pose, geometry_msgs.PoseWithCovarianceStamped):
        pose.pose.pose = naked_pose
    elif isinstance(pose, geometry_msgs.PoseWithCovariance):
        pose.pose = naked_pose
    elif isinstance(pose, geometry_msgs.PoseStamped):
        pose.pose = naked_pose
    elif isinstance(pose, geometry_msgs.Pose):
        pose = naked_pose
    else:
        raise ValueError("Input parameter pose is not any of geometry_msgs' poses!")
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


def yaw_diff(pose_or_quat_1, pose_or_quat_2):
    """ Normalized difference between the yaw of two poses or quaternions. """
    return angles_diff(yaw(pose_or_quat_1), yaw(pose_or_quat_2))


def heading(pose1, pose2=None):
    """ Heading angle from one pose to another.
        Poses are assumed to have the same reference frame. """
    if not pose2:
        pose2 = pose1
        pose1 = geometry_msgs.Pose()  # 0, 0, 0 pose, i.e. origin
    p1, p2 = __get_naked_poses(pose1, pose2)
    return np.arctan2(p2.position.y - p1.position.y, p2.position.x - p1.position.x)


def distance(x1, y1, x2, y2):
    """ Euclidean distance between 2D points """
    return np.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


def distance_2d(pose1, pose2=None):
    """ Euclidean distance between 2D poses; z coordinate is ignored.
        Poses are assumed to have the same reference frame. """
    if not pose2:
        pose2 = pose1
        pose1 = geometry_msgs.Pose()  # 0, 0, 0 pose, i.e. origin
    p1, p2 = __get_naked_poses(pose1, pose2)
    return np.sqrt(pow(p2.position.x - p1.position.x, 2)
                 + pow(p2.position.y - p1.position.y, 2))


def distance_3d(pose1, pose2=None):
    """ Euclidean distance between 3D poses.
        Poses are assumed to have the same reference frame. """
    if not pose2:
        pose2 = pose1
        pose1 = geometry_msgs.Pose()  # 0, 0, 0 pose, i.e. origin
    p1, p2 = __get_naked_poses(pose1, pose2)
    return np.sqrt(pow(p2.position.x - p1.position.x, 2)
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
        raise ValueError("Input parameter pose_or_quat is not a valid geometry_msgs object")

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
    x, y, z, w = quaternion_from_euler(0.0, 0.0, theta)
    return geometry_msgs.Quaternion(x=x, y=y, z=z, w=w)


def quaternion_msg_from_rpy(roll, pitch, yaw):
    """ Create a geometry_msgs/Quaternion from roll, pitch, yaw """
    x, y, z, w = quaternion_from_euler(roll, pitch, yaw)
    return geometry_msgs.Quaternion(x=x, y=y, z=z, w=w)


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
    point.point.x = float(x)
    point.point.y = float(y)
    point.point.z = float(z)
    if frame:
        point.header.frame_id = frame
        return point
    else:
        return point.point


def create_2d_pose(x, y, theta, frame=None):
    """ Create a geometry_msgs/Pose or geometry_msgs/PoseStamped
        (if frame is provided) from 2D coordinates and heading """
    pose = geometry_msgs.PoseStamped()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
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
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation = quaternion_msg_from_rpy(roll, pitch, yaw)
    if frame:
        pose.header.frame_id = frame
        return pose
    else:
        return pose.pose


def get_size_from_co(co):
    """ Get the size for a moveit_msgs/CollisionObject. We try first meshes, then primitives """
    if len(co.meshes):
        mesh = co.meshes[0]
        if len(mesh.vertices) < 2:
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
        raise ValueError("Input parameter pose is not a valid geometry_msgs pose object")
    return geometry_msgs.Pose2D(x=p.position.x, y=p.position.y, theta=yaw(p))


def to_pose3d(pose, timestamp=None, frame=None):
    """ timestamp is a builtin_interfaces/Time msg; zero if not provided """
    if isinstance(pose, geometry_msgs.Pose2D):
        p = geometry_msgs.Pose(position=geometry_msgs.Point(x=pose.x, y=pose.y, z=0.0),
                               orientation=quaternion_msg_from_yaw(pose.theta))
        if not frame:
            return p
        header = std_msgs.Header(frame_id=frame)
        if timestamp:
            header.stamp = timestamp
        return geometry_msgs.PoseStamped(header=header, pose=p)
    raise ValueError("Input parameter pose is not a geometry_msgs.Pose2D object")


def to_transform(pose, child_frame=None):
    if isinstance(pose, geometry_msgs.Pose2D):
        return geometry_msgs.Transform(translation=geometry_msgs.Vector3(x=pose.x, y=pose.y, z=0.0),
                                       rotation=quaternion_msg_from_yaw(pose.theta))
    elif isinstance(pose, geometry_msgs.Pose):
        return geometry_msgs.Transform(translation=geometry_msgs.Vector3(x=pose.position.x, y=pose.position.y,
                                                                         z=pose.position.z),
                                       rotation=pose.orientation)
    elif isinstance(pose, geometry_msgs.PoseStamped):
        p = pose.pose
        tf = geometry_msgs.Transform(translation=geometry_msgs.Vector3(x=p.position.x, y=p.position.y,
                                                                       z=p.position.z),
                                     rotation=p.orientation)
        return geometry_msgs.TransformStamped(header=pose.header, child_frame_id=child_frame or '', transform=tf)

    raise ValueError("Input parameter pose is not a valid geometry_msgs pose object")


def point2d2str(point):
    """ Provide a string representation of a geometry_msgs 2D point """
    if isinstance(point, geometry_msgs.Point):
        p = point
        f = ''
    elif isinstance(point, geometry_msgs.PointStamped):
        p = point.point
        f = ', ' + point.header.frame_id
    else:
        raise ValueError("Input parameter point is not a valid geometry_msgs point object")
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
        raise ValueError("Input parameter point is not a valid geometry_msgs point object")
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
        raise ValueError("Input parameter pose is not a valid geometry_msgs pose object")
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
        raise ValueError("Input parameter pose is not a valid geometry_msgs pose object")
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
        theta = pi / 2
    elif axis_or_theta == 'z':
        p.position.z += delta
        return __set_naked_pose(pose, p)
    else:
        raise ValueError(axis_or_theta + " is neither a number nor a valid axis ('x', 'y' or 'z')")
    if relative:
        theta = norm_angle(theta + yaw(p))
    p.position.x += np.cos(theta) * delta
    p.position.y += np.sin(theta) * delta
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
        raise ValueError(euler + " is not a valid euler angle (roll, pitch or yaw)")
    return pose


def transform_pose(pose, tf):
    """ Transform the given pose with the given transform """
    # do_transform_pose expects a stamped pose, but it ignores the header
    if isinstance(pose, geometry_msgs.Pose2D):
        p = to_pose3d(pose, frame='dummy')  # just force to_pose3d return a stamped pose
    elif isinstance(pose, geometry_msgs.Pose):
        p = geometry_msgs.PoseStamped(pose=pose)
    elif isinstance(pose, geometry_msgs.PoseStamped):
        p = pose
    else:
        raise ValueError("Input parameter pose is not a valid geometry_msgs pose object")

    return tf2_geometry_msgs.do_transform_pose_stamped(p, tf)


def transform_point(point, tf):
    """ Transform the given point with the given transform """
    # do_transform_point expects a stamped point, but it ignores the header
    if isinstance(point, geometry_msgs.Point):
        p = geometry_msgs.PointStamped(point=point)
    elif isinstance(point, geometry_msgs.PointStamped):
        p = point
    else:
        raise ValueError("Input parameter point is not a valid geometry_msgs point object")

    return tf2_geometry_msgs.do_transform_point(p, tf)


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


def calculate_velocity(points):
    """
    Calculate velocities between consecutive poses and return the average.
    """
    velocities = []
    for i in range(1, len(points)):
        delta_time = (Time.from_msg(points[i].header.stamp)
                      - Time.from_msg(points[i - 1].header.stamp)).nanoseconds * 1e-9
        if delta_time == 0:
            continue
        dx = points[i].point.x - points[i - 1].point.x
        dy = points[i].point.y - points[i - 1].point.y
        velocity = np.sqrt(dx ** 2 + dy ** 2) / delta_time
        velocities.append(velocity)
    return abs(np.mean(velocities)) if velocities else 0


def calculate_direction(points):
    if len(points) < 2:
        return 0
    dx = points[-1].point.x - points[0].point.x
    dy = points[-1].point.y - points[0].point.y
    return np.arctan2(dy, dx)


def project_future_pose(points, future_time):
    """
    Project the future pose using the list of last known positions.
    """
    velocity = calculate_velocity(points)
    heading = calculate_direction(points)

    future_point = geometry_msgs.Point()
    future_point.x = points[-1].point.x + np.cos(heading) * velocity * future_time
    future_point.y = points[-1].point.y + np.sin(heading) * velocity * future_time
    future_point.z = points[-1].point.z  # Assume the same z for 2D projection

    future_pose = geometry_msgs.PoseStamped()
    future_pose.header = points[-1].header
    future_pose.pose.position = future_point
    future_pose.pose.orientation = quaternion_msg_from_yaw(heading)

    return future_pose


class TF2(metaclass=Singleton):
    def __init__(self):
        """ Singleton encapsulating a tf2 listener and a broadcaster, using the node given to thorp_toolkit.init """
        self.__buff__ = tf2_ros.Buffer(node=node())
        # the listener gets its own node, spun on a dedicated thread; spinning ours there would take it
        # from the application's executor
        self.__list__ = tf2_ros.TransformListener(self.__buff__, None, spin_thread=True)
        self.__stbc__ = tf2_ros.StaticTransformBroadcaster(node())
        # wait until we get the first tf msg
        while rclpy.ok() and not self.__buff__.all_frames_as_string():
            time.sleep(0.001)

    def transform_pose(self, pose_in, frame_from, frame_to, timeout=Duration(seconds=2.0)):
        """
        Transform pose_in from one frame to another, or create
        the corresponding pose if None is provided on pose_in
        """
        if pose_in:
            frame_from = pose_in.header.frame_id
        else:
            pose_in = geometry_msgs.PoseStamped()
            pose_in.header.frame_id = frame_from
            pose_in.pose.orientation.w = 1.0
        try:
            return self.__buff__.transform(pose_in, frame_to, timeout)
        except tf2_ros.TransformException as err:
            raise RuntimeError(f"Could not transform pose from {frame_from} to {frame_to}: {err}")

    def lookup_transform(self, frame_from, frame_to, timestamp=Time(), timeout=Duration(seconds=1.0)):
        try:
            return self.__buff__.lookup_transform(frame_to, frame_from, timestamp, timeout)
        except tf2_ros.TransformException as err:
            raise RuntimeError(f"Could not lookup transform from {frame_from} to {frame_to}: {err}")

    def publish_transform(self, transform):
        self.__stbc__.sendTransform(transform)
