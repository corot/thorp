#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Trigger, TriggerResponse


class FOVAnalyzer:
    def __init__(self):
        self.bridge = CvBridge()
        self.min_distance = rospy.get_param('~min_distance', 0.4)
        self.crop_percentage = rospy.get_param('~crop_percentage', 0.20)  # crop 20 % bottom

        # Subscriber to the depth image topic
        rospy.Subscriber("xtion/depth_registered/image_raw", Image, self.depth_callback)

        # Service to check if there is anything closer than the specified distance blocking the fov
        self.service = rospy.Service("~check_clear", Trigger, self.check_clear_cb)

        self.depth_image = None

    def depth_callback(self, msg):
        self.depth_image = msg

    def check_clear_cb(self, req):
        if self.depth_image is None:
            rospy.logwarn("No depth image received yet; considering fov as blocked")
            return TriggerResponse(True, "No depth image received yet")

        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.depth_image, "32FC1")

            # Crop a percentage of pixels from the bottom
            if self.crop_percentage > 0:
                height = cv_img.shape[0]
                pixels_to_crop = int(self.crop_percentage * height)
                cv_img = cv_img[:-pixels_to_crop, :]
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert depth image: {e}")
            return TriggerResponse(True, "Could not convert depth image")

        # Check if any value in the depth image is less than the minimum distance
        if np.any((cv_img > 0) & (cv_img < self.min_distance)):
            return TriggerResponse(False, f"Object detected within {self.min_distance} m")

        return TriggerResponse(True, f"No objects detected within {self.min_distance} m")


if __name__ == "__main__":
    rospy.init_node("xtion_fov_analyzer")
    FOVAnalyzer()
    rospy.spin()
