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

import collections

from copy import deepcopy
from cv_bridge import CvBridge
from thorp_toolkit.geometry import distance_3d

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from cob_perception_msgs.msg import DetectionArray, Detection


class ObjectTrackingNode(object):
    """Get 3D values of bounding boxes returned by object recognizer node.

    _bridge (CvBridge): Bridge between ROS and CV image
    pub (Publisher): Publisher object for face depth results
    f (Float): Focal Length
    cx (Int): Principle Point Horizontal
    cy (Int): Principle Point Vertical

    """
    def __init__(self):
        # init the node
        rospy.init_node('cob_object_tracking', anonymous=False)

        self._buffer_length = 4   # TODO param  should be discard_after * freq
        self._discard_after = rospy.Duration(0.5)
        self._tracked_objs = {}
        self._target_objs = rospy.get_param('~target_objects', None)
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
        self._target_pub = rospy.Publisher('target_object_pose', PoseStamped, queue_size=1)
        self._tf2_bcaster = tf2_ros.TransformBroadcaster()

    def shutdown(self):
        """
        Shuts down the node
        """
        rospy.signal_shutdown("See ya!")

    def detection_callback(self, msg):
        """
        Callback for recognized objects

        Args:
        msg (cob_perception_msgs/DetectionArray): detections array
        """
        # Check if there is a detection
        for i, detection in enumerate(msg.detections):
            if detection.id not in self._tracked_objs:
                self._tracked_objs[detection.id] = collections.deque(maxlen=self._buffer_length)
            self._tracked_objs[detection.id].append(detection)

    def image_callback(self, msg):
        """
        Callback for RGB images showing recognized objects

        Args:
        msg (sensor_msgs/Image): RGB image from camera
        """
        self._last_img_cv = self._cv_bridge.imgmsg_to_cv2(msg)

    def spin(self):
        """
        Main loop: review tracked object observations and publish markers for them.
        Publish also the pose of our target object, if any
        """

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            da = DetectionArray()
            ma = MarkerArray()

            target_candidates = []
            for obs_buffer in self._tracked_objs.values():
                # clear old observations
                while obs_buffer:
                    if rospy.get_rostime() - obs_buffer[0].header.stamp <= self._discard_after:
                        break
                    obs_buffer.popleft()

                if len(obs_buffer) > self._buffer_length / 2:
                    # TODO: do smarter!  estimate prob
                    obs = deepcopy(obs_buffer[-1])
                    da.detections.append(obs)
                    ma.markers.append(self.make_detection_marker(obs))
                    self.pub_transform(obs)
                    self.pub_img_quad(obs)
                    if obs.label in self._target_objs:
                        target_candidates.append((obs.pose, distance_3d(obs.pose.pose)))

            if target_candidates:
                sorted(target_candidates, key=lambda x: x[1], reverse=True)
                self._target_pub.publish(target_candidates[0][0])
                ma.markers.append(self.make_target_marker(target_candidates[0][0]))

            if da.detections:
                self._objects_pub.publish(da)
                self._markers_pub.publish(ma)

            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                pass

    def make_target_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'target'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.1)  # short lived to make easy to figure out when we have a target
        marker.scale.x = 0.75
        marker.scale.y = 0.75
        marker.scale.z = 0.75
        marker.color.r = 1.0
        marker.color.a = 0.25
        marker.pose = deepcopy(pose.pose)
        ####marker.pose.position.y -= 0.5  # note it comes on camera optical frame
        # TODO would be better to use bounding_box_lwh.z / 2, but now is always zero!
        return marker

    def make_detection_marker(self, detection):
        marker = Marker()
        marker.header = detection.header
        marker.ns = str(detection.id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = detection.label + '_' + str(detection.id)
        marker.action = Marker.ADD
        marker.lifetime = self._discard_after
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.pose = deepcopy(detection.pose.pose)
        marker.pose.position.y -= 0.5  # note it comes on camera optical frame
        # TODO would be better to use bounding_box_lwh.z / 2, but now is always zero!
        return marker

    def pub_transform(self, detection):
        ObjectTrackingNode.prev = None  # hack to avoid the hated TF_REPEATED_DATA;  maybe makes more sense to not use buffer and pub each observation once
        tfs = TransformStamped()
        tfs.header = detection.header
        tfs.child_frame_id = detection.label + '_' + str(detection.id)
        tfs.transform.translation = detection.pose.pose.position
        tfs.transform.rotation = detection.pose.pose.orientation
        if not ObjectTrackingNode.prev:
            ObjectTrackingNode.prev = tfs
        elif ObjectTrackingNode.prev.header.stamp != tfs.header.stamp or ObjectTrackingNode.prev.child_frame_id != tfs.child_frame_id:
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
