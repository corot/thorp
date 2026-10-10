#!/usr/bin/env python3
"""
Measure the exploration coverage actually achieved: the map cells the camera has seen as the robot moves, with the
same field of view and line of sight model as Thorp's exploration planner, at the robot's actual pose.

Publishes, latched:
- coverage/explored (nav_msgs/OccupancyGrid): 0 for the cells seen, 100 for those the camera could see from somewhere
  but hasn't yet, unknown for the rest; show it on RViz as a Map with some transparency.
- coverage/percentage (std_msgs/Float32): seen fraction of what the camera could see from anywhere, in percent.
- rviz/coverage_overlay (rviz_2d_overlay_msgs/OverlayText): both percentages, as a text overlay.
On exit (Ctrl-C), saves an image of the explored map and the robot's trajectory, and prints the final figures.

    ros2 run thorp_exploration coverage_monitor.py --camera xtion --output /tmp/xtion --ros-args -p use_sim_time:=true
"""

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA, Float32
from tf2_ros import Buffer, TransformException, TransformListener

from thorp_exploration.planner import Camera, Grid, Roadmap


def field_of_view(camera):
    """ The camera's field of view polygon, from the exploration planner's parameters """
    path = os.path.join(get_package_share_directory('thorp_exploration'), 'param', 'exploration.yaml')
    params = yaml.safe_load(open(path))['exploration']['ros__parameters']
    fov = params[camera + '_field_of_view']
    return list(zip(fov[0::2], fov[1::2])), params


class FieldOfView:
    """ Cells within the field of view at any heading, on the square around the robot that Camera.visible returns """

    def __init__(self, polygon, camera):
        polygon = np.asarray(polygon, float)
        if np.cross(polygon[1] - polygon[0], polygon[2] - polygon[1]) < 0:
            polygon = polygon[::-1]
        self.polygon = polygon
        offsets = np.arange(-camera.radius, camera.radius + 1) * camera.resolution
        self.dx, self.dy = np.meshgrid(offsets, offsets)

    def view(self, heading):
        x = self.dx * math.cos(heading) + self.dy * math.sin(heading)
        y = -self.dx * math.sin(heading) + self.dy * math.cos(heading)
        inside = np.ones(self.dx.shape, bool)
        for a, b in zip(self.polygon, np.roll(self.polygon, -1, axis=0)):
            inside &= (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) >= 0
        return inside


class CoverageMonitor(Node):

    def __init__(self, args):
        super().__init__('coverage_monitor')
        self.args = args
        self.polygon, self.params = field_of_view(args.camera)
        self.grid = None
        self.seen = None
        self.visible_anywhere = None
        self.trajectory = []
        self.start_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, args.map_topic, self.map_cb, latched)
        self.explored_pub = self.create_publisher(OccupancyGrid, 'coverage/explored', latched)
        self.percentage_pub = self.create_publisher(Float32, 'coverage/percentage', latched)
        self.overlay_pub = self.create_publisher(OverlayText, 'rviz/coverage_overlay', latched)
        self.create_timer(1.0 / args.rate, self.sample)
        self.create_timer(1.0, self.publish)
        self.last_log = 0.0
        self.get_logger().info(f"Measuring coverage for the {args.camera}'s field of view; waiting for the map")

    def map_cb(self, msg):
        if self.grid is not None:
            return
        info = msg.info
        data = np.asarray(msg.data, np.int16).reshape(info.height, info.width)
        self.map_msg = msg
        self.grid = Grid(data, info.resolution, (info.origin.position.x, info.origin.position.y))
        self.camera = Camera(self.polygon, info.resolution, 1)
        self.camera.resolution = info.resolution
        self.fov = FieldOfView(self.polygon, self.camera)
        self.seen = np.zeros(self.grid.shape, bool)
        self.get_logger().info(f'Map of {info.width} x {info.height} cells received')

    def compute_visible_anywhere(self, x, y):
        """
        What the camera can see from the poses the planner considers, as its coverage target: cells with the viewpoint
        clearance, reachable from the robot's first pose, every viewpoint spacing, at any heading
        """
        started = time.time()
        roadmap = Roadmap(self.grid, self.params['clearance'])
        distances = roadmap.distances([roadmap.node_at(*self.grid.to_cell(x, y))])[0]
        reachable = np.zeros(self.grid.free.size, bool)
        reachable[np.flatnonzero(roadmap.traversable)[np.isfinite(distances)]] = True
        candidates = reachable.reshape(self.grid.shape) & (self.grid.clearance >= self.params['viewpoint_clearance'])
        step = max(1, int(round(self.params['viewpoint_spacing'] / self.grid.resolution)))
        any_heading = np.zeros(self.fov.dx.shape, bool)
        for heading in np.arange(36) * math.pi / 18:
            any_heading |= self.fov.view(heading)
        self.visible_anywhere = np.zeros(self.grid.shape, bool)
        rows, cols = np.nonzero(candidates)
        keep = (rows % step == 0) & (cols % step == 0)
        for row, col in zip(rows[keep], cols[keep]):
            self.mark(self.visible_anywhere, row, col, self.camera.visible(self.grid, row, col) & any_heading)
        self.visible_anywhere &= self.grid.free
        self.get_logger().info(f'{self.visible_anywhere.sum()} of {self.grid.free.sum()} free cells visible from '
                               f'{keep.sum()} candidate poses ({time.time() - started:.1f} s)')

    def mark(self, target, row, col, square):
        r = self.camera.radius
        rows, cols = np.nonzero(square)
        rows, cols = rows + row - r, cols + col - r
        inside = (rows >= 0) & (rows < self.grid.shape[0]) & (cols >= 0) & (cols < self.grid.shape[1])
        target[rows[inside], cols[inside]] = True

    def sample(self):
        if self.grid is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.map_msg.header.frame_id, self.args.robot_frame, Time(),
                                                 Duration(seconds=0.1))
        except TransformException:
            return
        x, y = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.visible_anywhere is None:
            self.compute_visible_anywhere(x, y)
            self.start_time = self.get_clock().now()
        row, col = self.grid.to_cell(x, y)
        if not (0 <= row < self.grid.shape[0] and 0 <= col < self.grid.shape[1]):
            return
        self.trajectory.append((x, y, yaw))
        self.mark(self.seen, row, col, self.camera.visible(self.grid, row, col) & self.fov.view(yaw))

    def figures(self):
        seen = self.seen & self.grid.free
        of_visible = 100.0 * (seen & self.visible_anywhere).sum() / max(1, self.visible_anywhere.sum())
        of_free = 100.0 * seen.sum() / max(1, self.grid.free.sum())
        distance = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.trajectory, self.trajectory[1:]))
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 if self.start_time else 0.0
        return of_visible, of_free, distance, elapsed

    def publish(self):
        if self.visible_anywhere is None:
            return
        of_visible, of_free, distance, elapsed = self.figures()
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_msg.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_msg.info
        data = np.full(self.grid.shape, -1, np.int8)
        data[self.visible_anywhere] = 100
        data[self.seen & self.grid.free] = 0
        msg.data = data.ravel().tolist()
        self.explored_pub.publish(msg)
        self.percentage_pub.publish(Float32(data=float(of_visible)))
        overlay = OverlayText()
        overlay.action = OverlayText.ADD
        overlay.width, overlay.height = 1000, 25
        overlay.horizontal_alignment, overlay.vertical_alignment = OverlayText.LEFT, OverlayText.TOP
        overlay.horizontal_distance, overlay.vertical_distance = 5, 70
        overlay.fg_color = ColorRGBA(r=0.1, g=1.0, b=0.4, a=1.0)
        overlay.text = f'Coverage {of_visible:.1f}% of visible, {of_free:.1f}% of free'
        overlay.text_size = 12.0
        self.overlay_pub.publish(overlay)
        now = time.time()
        if now - self.last_log >= self.args.log_period:
            self.last_log = now
            self.get_logger().info(f'Coverage {of_visible:.1f}% of visible, {of_free:.1f}% of free; '
                                   f'{distance:.1f} m in {elapsed:.0f} s')

    def save(self):
        if self.visible_anywhere is None:
            return
        of_visible, of_free, distance, elapsed = self.figures()
        image = np.full(self.grid.shape + (3,), 128, np.uint8)  # unknown or occupied: grey
        image[self.grid.free] = (255, 255, 255)
        image[self.visible_anywhere & ~self.seen] = (60, 60, 230)  # visible but not seen: red
        image[self.seen & self.grid.free] = (120, 220, 120)  # seen: green
        points = [self.grid.to_cell(x, y) for x, y, _ in self.trajectory]
        for (r0, c0), (r1, c1) in zip(points, points[1:]):
            cv2.line(image, (c0, r0), (c1, r1), (200, 0, 0), 1)
        image = cv2.flip(image, 0)  # row 0 is the map origin, at the bottom
        base = self.args.output
        cv2.imwrite(base + '.png', image)
        np.savez_compressed(base + '.npz', seen=self.seen, visible_anywhere=self.visible_anywhere,
                            free=self.grid.free, trajectory=np.asarray(self.trajectory))
        summary = (f'camera {self.args.camera}: coverage {of_visible:.1f}% of visible '
                   f'({(self.seen & self.visible_anywhere).sum()} of {self.visible_anywhere.sum()} cells), '
                   f'{of_free:.1f}% of free; traveled {distance:.1f} m in {elapsed:.0f} s')
        with open(base + '.txt', 'w') as f:
            f.write(summary + '\n')
        print(summary)
        print(f'saved {base}.png, .npz and .txt')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', default='kinect', choices=['kinect', 'xtion'])
    parser.add_argument('--rate', type=float, default=10.0, help='robot pose samples per second')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--robot-frame', default='base_footprint')
    parser.add_argument('--log-period', type=float, default=30.0, help='seconds between logged figures')
    parser.add_argument('--output', default='coverage', help='path and base name of the files saved on exit')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = CoverageMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
