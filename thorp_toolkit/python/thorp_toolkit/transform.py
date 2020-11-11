#!/usr/bin/env python

import copy
import numpy.linalg
from numpy.linalg import norm
import geometry_msgs.msg
import std_msgs.msg
import tf.transformations

import rospy


class Transform(object):
    """
    Represents a transform.
     - multiplication via * operator
     - inverse
     - convert to ros pose and transform geometry messages
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, frame_id=None):
        super(Transform, self).__init__()
        assert not frame_id or isinstance(frame_id, str)

        self.translation = [x, y, z]
        self.q = [qx, qy, qz, qw]

        self.header = std_msgs.msg.Header()
        if frame_id:
            self.header.frame_id = frame_id

        self.child_frame_id = None

        if not self._is_q_normalized():
            raise AssertionError("Quaternion not normalized in Transformation.__init__")

        try:
            _ = float(x)
        except ValueError:
            raise AssertionError("You most probably want to use Transformation.create!")

    @classmethod
    def create(cls, data):
        res = cls()
        if isinstance(data, geometry_msgs.msg.PoseStamped):
            res.header = copy.deepcopy(data.header)
            return res.from_geometry_msg_pose(data.pose)
        if isinstance(data, geometry_msgs.msg.PoseWithCovarianceStamped):
            res.header = copy.deepcopy(data.header)
            return res.from_geometry_msg_pose(data.pose.pose)
        elif isinstance(data, geometry_msgs.msg.Point):
            res.q = [0, 0, 0, 1]
            res.translation = [data.x, data.y, data.z]
            return res
        elif isinstance(data, geometry_msgs.msg.TransformStamped):
            res.header = copy.deepcopy(data.header)
            res.child_frame_id = copy.deepcopy(data.child_frame_id)
            return res.from_geometry_msg_transform(data.transform)
        elif isinstance(data, Transform):
            return copy.deepcopy(data)
        elif isinstance(data, geometry_msgs.msg.Pose):
            return res.from_geometry_msg_pose(data)
        elif isinstance(data, geometry_msgs.msg.PoseWithCovariance):
            return res.from_geometry_msg_pose(data.pose)
        elif isinstance(data, geometry_msgs.msg.Transform):
            return res.from_geometry_msg_transform(data)
        elif isinstance(data, dict):
            return res._from_dict(data)
        else:
            error_msg = ("Initialization of a 'Transformation' with arg of " +
                         "type '{}' is not supported!").format(type(data))
            raise TypeError(error_msg)

    @property
    def x(self):
        return self.translation[0]

    @x.setter
    def x(self, value):
        self.translation[0] = value

    @property
    def y(self):
        return self.translation[1]

    @y.setter
    def y(self, value):
        self.translation[1] = value

    @property
    def z(self):
        return self.translation[2]

    @z.setter
    def z(self, value):
        self.translation[2] = value

    @property
    def qx(self):
        return self.q[0]

    @qx.setter
    def qx(self, value):
        self.q[0] = value

    @property
    def qy(self):
        return self.q[1]

    @qy.setter
    def qy(self, value):
        self.q[1] = value

    @property
    def qz(self):
        return self.q[2]

    @qz.setter
    def qz(self, value):
        self.q[2] = value

    @property
    def qw(self):
        return self.q[3]

    @qw.setter
    def qw(self, value):
        self.q[3] = value

    @property
    def roll(self):
        return self.get_euler()[0]

    @property
    def pitch(self):
        return self.get_euler()[1]

    @property
    def yaw(self):
        return self.get_euler()[2]

    @property
    def frame_id(self):
        return self.header.frame_id

    @frame_id.setter
    def frame_id(self, value):
        self.header.frame_id = value

    def rotation_from_euler(self, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Set orientation form euler roll pitch yaw
        :param roll:
        :param pitch:
        :param yaw:
        :return:
        """
        self.q = tf.transformations.quaternion_from_euler(roll, pitch, yaw).tolist()
        return self

    def scale_translation(self, factor):
        """
        Scale the translational part by the given factor.
        If factor is an iterable of length 3 each axis will be scaled individually.
        :param factor: Scalar or of length 3
        :return:
        """
        try:
            _ = (f for f in factor)
            if len(factor) != 3:
                raise ValueError("Scaling iterable has to be of length 3")
        except TypeError:
            factor = (factor, factor, factor)
        self.translation = [t * f for t, f in zip(self.translation, factor)]

    def split(self, translation_first=True):
        """
        Splits the Transformation into a translational and a rotational Transformation in such a way that multiplying
        the two return values 'first'*'second' always yields the original Transformation.
        :param translation_first: If true (default) 'first' will be the translational component. Consequently
        first.translation == self.translation and second.rotation == self.rotation.
        Otherwise first.rotation = self.rotation, but second.translation != self.translation!
        :return:
        """
        translation = Transform(*self.translation)
        rotation = Transform.create(self)
        rotation.translation = [0, 0, 0]

        if translation_first:
            return translation, rotation
        else:
            return rotation, rotation.inverse() * translation

    def to_matrix(self):
        matrix = tf.transformations.quaternion_matrix(self.q)
        matrix[:3, 3] = self.translation[:3]
        return matrix

    def from_matrix(self, matrix):
        self.translation = matrix[:3, 3]
        self.q = tf.transformations.quaternion_from_matrix(matrix).tolist()
        return self

    def inverse(self):
        t = Transform().from_matrix(numpy.linalg.inv(self.to_matrix()))
        t.header = copy.deepcopy(self.header)
        t.header.frame_id = copy.deepcopy(self.child_frame_id)
        t.child_frame_id = copy.deepcopy(self.header.frame_id)
        return t

    def get_quaternion(self):
        """
        :return: geometry_msgs.msg.Quaternion
        """
        quaternion = geometry_msgs.msg.Quaternion()
        assert self._is_q_normalized()
        quaternion.x = self.q[0]
        quaternion.y = self.q[1]
        quaternion.z = self.q[2]
        quaternion.w = self.q[3]
        return quaternion

    def _is_q_normalized(self):
        return abs(1 - norm(self.q)) < 1e-5

    def get_euler(self):
        """
        :return: [r,p,y]
        """
        return tf.transformations.euler_from_quaternion(self.q)

    def norm_translation(self):
        return self.length()

    def get_vector3(self):
        """
        :return: geometry_msgs.msg.Vector3
        """
        vector3 = geometry_msgs.msg.Vector3()
        vector3.x = self.translation[0]
        vector3.y = self.translation[1]
        vector3.z = self.translation[2]
        return vector3

    def to_geometry_msg_transform(self):
        """
        :return: geometry_msgs.msg.Transform
        """
        msg = geometry_msgs.msg.Transform()
        msg.translation.x = self.translation[0]
        msg.translation.y = self.translation[1]
        msg.translation.z = self.translation[2]
        msg.rotation.x = self.q[0]
        msg.rotation.y = self.q[1]
        msg.rotation.z = self.q[2]
        msg.rotation.w = self.q[3]
        return msg

    def to_geometry_msg_transform_stamped(self):
        msg = geometry_msgs.msg.TransformStamped()
        msg.transform = self.to_geometry_msg_transform()
        msg.header = copy.deepcopy(self.header)
        if self.child_frame_id:
            msg.child_frame_id = self.child_frame_id
        return msg

    def from_geometry_msg_transform(self, msg):
        """
        :param msg
        :return: geometry_msgs.msg.Transform
        """
        self.translation[0] = msg.translation.x
        self.translation[1] = msg.translation.y
        self.translation[2] = msg.translation.z
        self.q[0] = msg.rotation.x
        self.q[1] = msg.rotation.y
        self.q[2] = msg.rotation.z
        self.q[3] = msg.rotation.w
        return self

    def to_geometry_msg_pose(self):
        """
        :return:  geometry_msgs.msg.Pose
        """
        msg = geometry_msgs.msg.Pose()
        msg.position.x = self.translation[0]
        msg.position.y = self.translation[1]
        msg.position.z = self.translation[2]
        msg.orientation.x = self.q[0]
        msg.orientation.y = self.q[1]
        msg.orientation.z = self.q[2]
        msg.orientation.w = self.q[3]
        return msg

    def to_geometry_msg_pose_stamped(self):
        msg = geometry_msgs.msg.PoseStamped()
        msg.header = copy.deepcopy(self.header)
        msg.pose = self.to_geometry_msg_pose()
        return msg

    def from_geometry_msg_pose(self, msg):
        """
        :param msg: geometry_msgs.msg.Pose
        :return: None
        """
        self.translation[0] = msg.position.x
        self.translation[1] = msg.position.y
        self.translation[2] = msg.position.z
        self.q[0] = msg.orientation.x
        self.q[1] = msg.orientation.y
        self.q[2] = msg.orientation.z
        self.q[3] = msg.orientation.w
        return self

    def from_transform_stamped(self, stamped_transform):
        self.from_geometry_msg_transform(stamped_transform.transform)
        return self

    def to_dict(self):
        # Helper function to create cman-compatible dictionary, use e.g. like this:
        # upd = {trafo.child_frame_id: trafo._as_dict()}
        # update_config(cman_app, cman_file, upd)
        tl = self.translation
        r = self.get_euler()
        updatedict = {'frame_id': self.header.frame_id,
                      'translation': {'x': float(tl[0]), 'y': float(tl[1]), 'z': float(tl[2])},
                      'rotation': {'r': float(r[0]), 'p': float(r[1]), 'y': float(r[2])}}
        if self.child_frame_id:
            updatedict['child_frame_id'] = self.child_frame_id
        return updatedict

    @staticmethod
    def _from_dict(d):
        # is no member-function as operation can fail (e.g. d = {})
        try:
            frame_id = d['frame_id']
            tr = d['translation']
            t = map(float, [tr['x'], tr['y'], tr['z']])  # cman can return numpy ob
            q_d = d['rotation']

            t_foo = Transform()
            t_foo.rotation_from_euler(q_d['r'], q_d['p'], q_d['y'])
            q = list(t_foo.q)
        except KeyError:
            rospy.logerr("Could not parse '%s' into Transformation" % str(d))
            return None

        new_trafo = Transform(*(t + q + [frame_id]))
        new_trafo.child_frame_id = d.get('child_frame_id', None)

        return new_trafo

    def length(self):
        return float(norm(self.translation))

    def distance(self, other):
        delta = [x - y for x, y in zip(self.translation, other.translation)]
        return float(norm(delta))

    def __mul__(self, other):
        if not isinstance(other, Transform):
            raise TypeError
            
        l = self.to_matrix()
        r = other.to_matrix()
        result = Transform()
        result.from_matrix(tf.transformations.concatenate_matrices(l, r))
        result.header = copy.deepcopy(self.header)
        if self.child_frame_id and other.header.frame_id  and self.child_frame_id != other.header.frame_id:
            rospy.logwarn("Inconsistency in your transform chain: concatenated "
                           "a frame in %s to a frame which specifies %s",
                           other.header.frame_id, self.child_frame_id)
        result.child_frame_id = other.child_frame_id
        return result

    def __unicode__(self):
        euler = tf.transformations.euler_from_quaternion(self.q)
        if self.header.frame_id and self.child_frame_id:
            return "%f, %f, %f - %f, %f, %f in '%s ->%s'" % (self.translation[0], self.translation[1], self.translation[2],
                                                             euler[0], euler[1], euler[2], self.header.frame_id, self.child_frame_id)
        if self.header.frame_id:
            return "%f, %f, %f - %f, %f, %f in '%s'" % (self.translation[0], self.translation[1], self.translation[2],
                                                        euler[0], euler[1], euler[2], self.header.frame_id)
        else:
            return "%f, %f, %f - %f, %f, %f" % (self.translation[0], self.translation[1], self.translation[2],
                                                euler[0], euler[1], euler[2])

    def __str__(self):
        return unicode(self).encode('utf-8')

    def __repr__(self):
        return self.__str__()

    def __ne__(self, other):
        return not self == other

    def __eq__(self, other):
        if not isinstance(other, Transform):
            return False

        if self.header.frame_id and other.header.frame_id:
            if self.header.frame_id != other.header.frame_id:
                rospy.logerr("Header frames of Transformation objects do not fit (%s and %s), comparison"
                               "will fail in future version" % (self.header.frame_id, other.header.frame_id))
                # return False

        if self.child_frame_id and other.child_frame_id:
            if self.child_frame_id != other.child_frame_id:
                rospy.logerr("Child frames of Transformation objects do not fit (%s and %s), comparison" 
                               "will fail in future version" % (self.child_frame_id, other.child_frame_id))
                # return False

        for (a, b) in zip(list(self.translation) + list(self.q), list(other.translation) + list(other.q)):
            if abs(a - b) > 0.0001:
                return False
        return True
