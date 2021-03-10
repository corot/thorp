from math import sqrt
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from std_msgs.msg import Header
import numpy as np


class Transformer(object):
    @staticmethod
    def _p2q(pose):
        if sqrt(pose.pose.orientation.w * pose.pose.orientation.w +
                                pose.pose.orientation.z * pose.pose.orientation.z +
                                pose.pose.orientation.y * pose.pose.orientation.y +
                                pose.pose.orientation.x * pose.pose.orientation.x) < .9:
            return (1, 0, 0, 0)

        return (pose.pose.orientation.w, pose.pose.orientation.x,
                pose.pose.orientation.y, pose.pose.orientation.z)

    @staticmethod
    def _q_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        wt = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        xt = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        yt = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        zt = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        return wt, xt, yt, zt

    @staticmethod
    def _q_conjugate(q):
        w, x, y, z = q
        return w, -x, -y, -z

    @staticmethod
    def _transform_point(pose, point):
        quaternion = Transformer._p2q(pose)
        _, x, y, z = Transformer._q_multiply(
            Transformer._q_multiply(Transformer._q_conjugate(quaternion),
                                    point),
            quaternion)
        return x, y, z

    @staticmethod
    def _transform_point_q(quaternion, point):
        w0, x, y, z = Transformer._q_multiply(
            Transformer._q_multiply(Transformer._q_conjugate(quaternion),
                                    point),
            quaternion)
        if abs(w0) > 0.000000000001:
            print("w0 != 0")
            print(w0)
        return x, y, z

    @staticmethod
    def _p2p(pose):
        return (0, pose.pose.position.x,
                pose.pose.position.y, pose.pose.position.z)

    @staticmethod
    def _transform_rotation(pose, pose_to_trans):
        # print "pose: ", pose
        # print "pose2: ",pose_to_trans
        w, x, y, z = Transformer._q_multiply(Transformer._p2q(pose),
                                             Transformer._p2q(pose_to_trans))

        return w, x, y, z

    @staticmethod
    def apply_local_rotation(transform, transform_to_rotate):
        pose = Transformer.transform_to_pose(transform)
        pose_to_rotate = Transformer.transform_to_pose(transform)
        w, x, y, z = Transformer._transform_rotation(pose, pose_to_rotate)
        transform_to_rotate.rotation.w = w
        transform_to_rotate.rotation.x = x
        transform_to_rotate.rotation.y = y
        transform_to_rotate.rotation.z = z

    @staticmethod
    def _transform_rotation_q(quat1, quat2):
        # print "pose: ", pose
        # print "pose2: ",pose_to_trans
        w, x, y, z = Transformer._q_multiply(quat1, quat2)
        return w, x, y, z

    @staticmethod
    def transform_point_list(pose, point_list):
        for p in point_list:
            x, y, z = Transformer._transform_point(
                pose,
                (0, p.x - pose.pose.position.x, p.y - pose.pose.position.y, p.z - pose.pose.position.z))
            yield Point(x, y, z)

    @staticmethod
    def transform_point(pose, point):
        x, y, z = Transformer._transform_point(
            pose,
            (0, point.x - pose.pose.position.x, point.y - pose.pose.position.y, point.z - pose.pose.position.z))
        return Point(x, y, z)

    @staticmethod
    def combine_pose(pose1, pose2):
        x, y, z = Transformer._transform_point(
            Transformer.invert_pose(pose1, pose2.header.frame_id),
            Transformer._p2p(pose2))
        qw, qx, qy, qz = Transformer._transform_rotation(pose1, pose2)

        n = sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        qw /= n
        qx /= n
        qy /= n
        qz /= n
        pose_res = PoseStamped()
        pose_res.header.frame_id = pose1.header.frame_id
        pose_res.pose.position.x = x + pose1.pose.position.x
        pose_res.pose.position.y = y + pose1.pose.position.y
        pose_res.pose.position.z = z + pose1.pose.position.z

        pose_res.pose.orientation.w = qw
        pose_res.pose.orientation.x = qx
        pose_res.pose.orientation.y = qy
        pose_res.pose.orientation.z = qz

        return pose_res

    @staticmethod
    def invert_pose(pose1, new_frame):
        quaternion = Transformer._q_conjugate(Transformer._p2q(pose1))
        x, y, z = Transformer._transform_point_q(
            Transformer._p2q(pose1), Transformer._p2p(pose1))

        qw, qx, qy, qz = quaternion

        pose_res = PoseStamped()
        pose_res.header.frame_id = new_frame
        pose_res.pose.position.x = -x
        pose_res.pose.position.y = -y
        pose_res.pose.position.z = -z

        pose_res.pose.orientation.w = qw
        pose_res.pose.orientation.x = qx
        pose_res.pose.orientation.y = qy
        pose_res.pose.orientation.z = qz

        return pose_res

    @staticmethod
    def invert_transform(transform, header=Header(), child_frame_id=""):
        if type(transform) == TransformStamped:
            return Transformer.invert_transform(transform.transform, transform.header, transform.child_frame_id)
        pose = Transformer.transform_to_pose(transform, header)
        inverse_pose = Transformer.invert_pose(pose, child_frame_id)
        return Transformer.pose_to_transform(inverse_pose, header.frame_id)

    @staticmethod
    def transform_to_pose(transform, header=Header()):
        if type(transform) == TransformStamped:
            return Transformer.transform_to_pose(transform.transform, transform.header)
        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = header.frame_id
        pose.pose.position.x = transform.translation.x
        pose.pose.position.y = transform.translation.y
        pose.pose.position.z = transform.translation.z
        pose.pose.orientation.x = transform.rotation.x
        pose.pose.orientation.y = transform.rotation.y
        pose.pose.orientation.z = transform.rotation.z
        pose.pose.orientation.w = transform.rotation.w
        return pose

    @staticmethod
    def pose_to_transform(pose, child_frame_id):
        transform = TransformStamped()
        transform.header.stamp = pose.header.stamp
        transform.header.frame_id = pose.header.frame_id
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation.x = pose.pose.orientation.x
        transform.transform.rotation.y = pose.pose.orientation.y
        transform.transform.rotation.z = pose.pose.orientation.z
        transform.transform.rotation.w = pose.pose.orientation.w
        return transform

    @staticmethod
    def rot_q_around_axis(q, axis='z', degrees=90):
        if axis != 'z' or degrees != 90:
            print("NOT YET IMPLEMENTED")
            return
        q_rot = 1 / 2 ** 0.5, 0, 0, 1 / 2 ** 0.5

        qw, qx, qy, qz = Transformer._transform_rotation_q(q, q_rot)

        n = sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        qw /= n
        qx /= n
        qy /= n
        qz /= n
        return qw, qx, qy, qz

    @staticmethod
    def quaternion_from_axis_angle(direction_vector, rad):
        fac = np.sin(rad)

        x = direction_vector.x * fac
        y = direction_vector.y * fac
        z = direction_vector.z * fac

        w = np.cos(rad / 2.0)

        q_np = np.array([x, y, z, w])
        q_np = q_np / np.linalg.norm(q_np)
        return q_np
