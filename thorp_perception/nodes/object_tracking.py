#!/usr/bin/env python

"""
A ROS node that gathers information about general objects detected in the environment,
in opposition to tabletop objects for manipulation. For every object, we track for a
time before publishing, adding some information, e.g., moving, certitude, and so on.

Author:
    Jorge Santos
"""
import threading

import rospy
import tf2_ros
import numpy as np

from copy import deepcopy
from cv_bridge import CvBridge
from thorp_toolkit.geometry import TF2, distance_3d, create_2d_pose
from thorp_toolkit.point_tracker import PointTracker
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from cob_perception_msgs.msg import DetectionArray


class ObjectTrackingNode(object):
    def __init__(self):
        rospy.init_node('cob_object_tracking', anonymous=False)

        TF2()  # to start filling tf buffer

        self._discard_after = rospy.Duration(0.5)
        self._tracked_objs = {}
        self._trackers = {}
        self._tracked_lock = threading.Lock()
        self._target_objs = rospy.get_param('~target_objects', None)
        self._last_img_cv = None
        self._cv_bridge = CvBridge()

        # Subscribe to object detection and source image
        self._detect_sub = rospy.Subscriber('object_detection/detections/with_pose', DetectionArray,
                                            self.detection_callback, queue_size=1)
        self._image_sub = rospy.Subscriber('object_detection/detections_image', Image,
                                           self.image_callback, queue_size=1)

        # Advertise the results and markers for visualization on RViz
        self._tf2_bcaster = tf2_ros.TransformBroadcaster()
        self._objects_pub = rospy.Publisher('tracked_objects', DetectionArray, queue_size=1)
        self._markers_pub = rospy.Publisher('tracked_objects_markers', MarkerArray, queue_size=1)
        self._images_pub = rospy.Publisher('tracked_objects_images', Image, queue_size=1)
        self._target_pub = rospy.Publisher('target_object_pose', PoseStamped, queue_size=1)
        self._predicted_pub = rospy.Publisher('target_object_predicted_pose', PoseStamped, queue_size=1)

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
        with self._tracked_lock:
            # Check if there is a detection
            for detection in msg.detections:
                self._tracked_objs[detection.id] = detection

                pose_wrt_map = TF2().transform_pose(detection.pose, None, 'map')
                position_2d = pose_wrt_map.pose.position.x, pose_wrt_map.pose.position.y
                if detection.id not in self._trackers:
                    self._trackers[detection.id] = PointTracker(position_2d)
                else:
                    self._trackers[detection.id].update(position_2d)

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

            lost_objs = []
            target_candidates = []
            with self._tracked_lock:
                for obj_id, detection in self._tracked_objs.items():
                    # clear old observations
                    if rospy.get_rostime() - detection.header.stamp > self._discard_after:
                        lost_objs.append(obj_id)
                        continue

                    da.detections.append(detection)
                    ma.markers.append(self.make_detection_marker(detection))
                    self.pub_transform(detection)
                    self.pub_img_quad(detection)
                    if detection.label in self._target_objs:
                        target_candidates.append((detection, distance_3d(detection.pose)))

                for obj_id in lost_objs:
                    del self._tracked_objs[obj_id]
                    del self._trackers[obj_id]

            if target_candidates:
                target_candidates.sort(key=lambda x: x[1], reverse=True)
                target, distance = target_candidates[0]
                self._target_pub.publish(target.pose)
                ma.markers.append(self.make_target_marker(target.pose))

                dt = 5 # TODO:  pred speed kk   should be 0.01 x gazebo freq,  I guess 0.1    and its 0.005
                tracker = self._trackers[target.id]
                predicted_state = tracker.predict()
                predicted_x = predicted_state[0] + predicted_state[2] * dt
                predicted_y = predicted_state[1] + predicted_state[3] * dt
                predicted_t = predicted_state[4]
                predicted_v = np.hypot(predicted_state[2], predicted_state[3])
                future_pose = create_2d_pose(predicted_x, predicted_y, predicted_t, 'map')

                rospy.logdebug("Target direction: %f; speed: %f m/s", predicted_t, predicted_v)
                self._predicted_pub.publish(future_pose)

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
        marker.lifetime = rospy.Duration(0.1)  # short-lived to make it easy to figure out when we have a target
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
        ObjectTrackingNode.prev = None  # hack to avoid the hated TF_REPEATED_DATA; maybe makes more sense to not use buffer and pub each observation once
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
