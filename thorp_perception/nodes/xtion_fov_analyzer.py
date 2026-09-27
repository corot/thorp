#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class FOVAnalyzer(Node):
    """ Check whether something closer than a minimum distance blocks the Xtion field of view """

    def __init__(self):
        super().__init__('xtion_fov_analyzer')
        self.min_distance = self.declare_parameter('min_distance', 0.4).value
        self.crop_percentage = self.declare_parameter('crop_percentage', 0.20).value  # crop 20 % bottom
        self.depth_image = None
        self.create_subscription(Image, 'xtion/depth_registered/image_raw', self.depth_callback,
                                 rclpy.qos.qos_profile_sensor_data)
        self.create_service(Trigger, '~/check_clear', self.check_clear_cb)

    def depth_callback(self, msg):
        self.depth_image = msg

    def check_clear_cb(self, request, response):
        if self.depth_image is None:
            self.get_logger().warning("No depth image received yet; considering fov as clear")
            response.success, response.message = True, "No depth image received yet"
            return response
        if self.depth_image.encoding != '32FC1':
            self.get_logger().error(f"Unsupported depth image encoding {self.depth_image.encoding}")
            response.success, response.message = True, "Unsupported depth image encoding"
            return response
        depth = np.frombuffer(self.depth_image.data, dtype=np.float32).reshape(self.depth_image.height, -1)
        # Crop a percentage of pixels from the bottom
        depth = depth[:self.depth_image.height - int(self.crop_percentage * self.depth_image.height), :]
        # Check if any value in the depth image is less than the minimum distance
        if np.any((depth > 0) & (depth < self.min_distance)):
            response.success, response.message = False, f"Object detected within {self.min_distance} m"
        else:
            response.success, response.message = True, f"No objects detected within {self.min_distance} m"
        return response


def main():
    rclpy.init()
    node = FOVAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
