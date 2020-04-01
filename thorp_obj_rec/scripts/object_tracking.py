#!/usr/bin/env python
"""
A ROS node that gathers information about general objects detected in the environment,
in opposition to tabletop objects for manipulation. For evey object, we track for a
time before publishing, adding some information, e.g. moving, certitude and so.

Author:
    Jorge Santos
"""

import rospy
import tf2_ros
import tf_conversions

import numpy as np
import collections

from math import pi
from copy import deepcopy
from cv_bridge import CvBridge
from thorp_toolkit.geometry import transform_pose, heading

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, Pose
from visualization_msgs.msg import MarkerArray, Marker
from cob_perception_msgs.msg import DetectionArray, Detection


class ObjectTrackingNode(object):
    """Get 3D values of bounding boxes returned by face_recognizer node.

    _bridge (CvBridge): Bridge between ROS and CV image
    pub (Publisher): Publisher object for face depth results
    f (Float): Focal Length
    cx (Int): Principle Point Horizontal
    cy (Int): Principle Point Vertical

    """
    def __init__(self):
        # init the node
        rospy.init_node('cob_object_tracking', anonymous=False)

        self._buffer_length = 10   # TODO param  should be discard_after * freq
        self._discard_after = rospy.Duration(5)
        self._tracked_objs = {}
        self._last_img_cv = None
        self._cv_bridge = CvBridge()

        # Subscribe to object detection and source image
        self._detect_sub = rospy.Subscriber('object_detection/detections/with_pose', DetectionArray,
                                            self.detection_callback, queue_size=1)
        self._image_sub = rospy.Subscriber('object_detection/detections_image', Image,
                                           self.image_callback, queue_size=1)

        # Advertise the results and markers for visualization on RViz
        self._objects_pub = rospy.Publisher('tracked_objects', DetectionArray, queue_size=1)
        self._markers_pub = rospy.Publisher('tracked_objects_markers', MarkerArray, queue_size=1)
        self._images_pub = rospy.Publisher('tracked_objects_images', Image, queue_size=1)
        self._tf2_bcaster = tf2_ros.TransformBroadcaster()

    def shutdown(self):
        """
        Shuts down the node
        """
        rospy.signal_shutdown("See ya!")

    def detection_callback(self, msg):
        """
        Callback for RGB images: The main logic is applied here

        Args:
        msg (cob_perception_msgs/DetectionArray): detections array
        depth (sensor_msgs/PointCloud2): depth image from camera

        """
        # Check if there is a detection
        for i, detection in enumerate(msg.detections):
            if detection.id not in self._tracked_objs:
                self._tracked_objs[detection.id] = collections.deque(maxlen=self._buffer_length)
            self._tracked_objs[detection.id].append(detection)

    def image_callback(self, msg):
        self._last_img_cv = self._cv_bridge.imgmsg_to_cv2(msg)

    def spin(self):
        """
        Gets the necessary parameters from parameter server

        Returns:
        (tuple) :
            depth_topic (String): Incoming depth topic name
            face_topic (String): Incoming face bounding box topic name
            output_topic (String): Outgoing depth topic name
            f (Float): Focal Length
            cx (Int): Principle Point Horizontal
            cy (Int): Principle Point Vertical
        """

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            da = DetectionArray()
            ma = MarkerArray()

            for obs_buffer in self._tracked_objs.values():
                # clear old observations
                while obs_buffer:
                    if rospy.get_rostime() - obs_buffer[0].header.stamp <= self._discard_after:
                        break
                    obs_buffer.popleft()

                if len(obs_buffer) > self._buffer_length / 2:
                    # TODO: do smarter!  estimate prob
                    obs = deepcopy(obs_buffer[-1])
                    obs.pose = transform_pose('base_footprint', obs.pose)
                    obs.header = obs.pose.header
                    da.detections.append(obs)
                    ma.markers.append(self.make_marker(obs))
                    self.pub_transform(obs)
                    self.pub_img_quad(obs)

            if da.detections:
                self._objects_pub.publish(da)
                self._markers_pub.publish(ma)

            rate.sleep()

    def make_marker(self, detection):
        marker = Marker()
        marker.header = detection.header
        marker.ns = str(detection.id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = detection.label + '_' + str(detection.id)
        # marker.type = Marker.MESH_RESOURCE
        # marker.mesh_resource = "package://thorp_obj_rec/meshes/kitty.dae"
        marker.action = Marker.ADD
        marker.lifetime = self._discard_after
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.pose = deepcopy(detection.pose.pose)
        marker.pose.position.z += 0.5  # need bounding_box_lwh.z / 2,  but now is all zero!
        return marker

    def pub_transform(self, detection):
        tfs = TransformStamped()
        tfs.header = detection.header
        tfs.child_frame_id = detection.label + '_' + str(detection.id)
        tfs.transform.translation = detection.pose.pose.position
        # looking to the robot
        q = tf_conversions.transformations.quaternion_from_euler(-pi/2.0, 0.0, heading(detection.pose.pose) + pi/2.0)
        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]
        self._tf2_bcaster.sendTransform(tfs)

    def pub_img_quad(self, detection):
        img_quad = self._last_img_cv[detection.mask.roi.y:detection.mask.roi.y + detection.mask.roi.height,
                                     detection.mask.roi.x:detection.mask.roi.x + detection.mask.roi.width]
        img_quad = self._cv_bridge.cv2_to_imgmsg(img_quad, "rgb8")
        img_quad.header.stamp = detection.header.stamp
        img_quad.header.frame_id = detection.label + '_' + str(detection.id)
        self._images_pub.publish(img_quad)


if __name__ == '__main__':
    node = ObjectTrackingNode()
    node.spin()
