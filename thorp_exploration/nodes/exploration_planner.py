#!/usr/bin/env python3

"""
Exploration planner: segments the map into rooms, and plans the rooms' sequence and the poses from where the camera
sees all of a room, or of the whole map. Shows the rooms and the last poses planned as markers.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from thorp_msgs.msg import Room
from thorp_msgs.srv import PlanRoomExploration, PlanRoomSequence, SegmentRooms

from thorp_exploration.planner import Camera, Grid, Planner

# distinct colors for neighboring room ids
PALETTE = [(0.90, 0.10, 0.29), (0.24, 0.71, 0.29), (1.00, 0.88, 0.10), (0.26, 0.39, 0.85), (0.96, 0.51, 0.19),
           (0.57, 0.12, 0.71), (0.27, 0.94, 0.94), (0.94, 0.20, 0.90), (0.74, 0.96, 0.05), (0.98, 0.75, 0.83),
           (0.00, 0.50, 0.50), (0.60, 0.39, 0.16)]


class ExplorationPlanner(Node):
    def __init__(self):
        super().__init__('exploration')
        self.declare_parameter('camera', 'kinect')
        self.declare_parameter('kinect_field_of_view', [0.5, 0.289, 0.5, -0.289, 2.0, -1.155, 2.0, 1.155])
        self.declare_parameter('xtion_field_of_view', [0.2, 0.2, 0.2, -0.2, 1.2, -0.6, 1.2, 0.6])
        self.declare_parameter('clearance', 0.25)
        self.declare_parameter('viewpoint_clearance', 0.4)
        self.declare_parameter('room_area_min', 2.0)
        self.declare_parameter('room_area_max', 20.0)
        self.declare_parameter('viewpoint_spacing', 0.3)
        self.declare_parameter('headings', 8)
        self.declare_parameter('coverage', 0.95)
        self.declare_parameter('min_view_area', 0.25)
        camera = self.get_parameter('camera').value
        fov = self.get_parameter(camera + '_field_of_view').value
        self.field_of_view = list(zip(fov[0::2], fov[1::2]))

        self.planner = None
        self.frame = 'map'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.rooms_pub = self.create_publisher(MarkerArray, '~/rooms', latched)
        self.viewpoints_pub = self.create_publisher(MarkerArray, '~/viewpoints', latched)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, latched)
        self.create_service(SegmentRooms, '~/segment_rooms', self.segment_rooms)
        self.create_service(PlanRoomSequence, '~/plan_room_sequence', self.plan_room_sequence)
        self.create_service(PlanRoomExploration, '~/plan_room_exploration', self.plan_room_exploration)
        self.get_logger().info(f"Exploration planner ready, for the {camera} camera")

    def map_callback(self, msg):
        info = msg.info
        grid = Grid(np.array(msg.data, np.int8).reshape(info.height, info.width), info.resolution,
                    (info.origin.position.x, info.origin.position.y))
        param = lambda name: self.get_parameter(name).value  # noqa: E731
        self.planner = Planner(grid, Camera(self.field_of_view, info.resolution, param('headings')),
                               param('clearance'), param('viewpoint_clearance'), param('room_area_min'),
                               param('room_area_max'), param('viewpoint_spacing'), param('coverage'),
                               param('min_view_area'))
        self.frame = msg.header.frame_id
        self.get_logger().info(f"Map of {info.width} x {info.height} cells received")

    def segment_rooms(self, request, response):
        if self.check_planner(response):
            rooms = self.planner.segment_rooms()
            for room, area, (x, y) in rooms:
                response.rooms.append(Room(id=room, area=area, center=Point(x=x, y=y)))
            response.success = True
            self.get_logger().info(f"Map segmented into {len(rooms)} rooms")
            self.publish_rooms()
        return response

    def plan_room_sequence(self, request, response):
        if self.check_planner(response) and self.check_start(request.start, response):
            try:
                rooms = list(request.rooms) or list(self.planner.rooms)
                response.sequence = self.planner.plan_room_sequence(self.start_point(request.start), rooms)
                response.success = True
                self.get_logger().info(f"Room sequence: {list(response.sequence)}")
                if len(response.sequence) < len(rooms):
                    self.get_logger().warn(f"Rooms not reachable: {sorted(set(rooms) - set(response.sequence))}")
            except ValueError as e:
                self.fail(response, str(e))
        return response

    def plan_room_exploration(self, request, response):
        if self.check_planner(response) and self.check_start(request.start, response):
            try:
                poses = self.planner.plan_room_exploration(self.start_point(request.start), request.room)
                response.poses = [self.pose(x, y, yaw) for x, y, yaw in poses]
                response.success = True
                self.get_logger().info(f"{len(poses)} poses to explore "
                                       f"{'room %d' % request.room if request.room else 'the whole map'}")
                self.publish_viewpoints(request.start, poses)
            except ValueError as e:
                self.fail(response, str(e))
        return response

    def check_planner(self, response):
        if self.planner is None:
            return self.fail(response, "no map received yet")
        return True

    def check_start(self, start, response):
        if start.header.frame_id.lstrip('/') != self.frame:
            return self.fail(response, f"start pose must be on the {self.frame} frame, not '{start.header.frame_id}'")
        return True

    def fail(self, response, message):
        response.success = False
        response.message = message
        self.get_logger().error(message)
        return False

    @staticmethod
    def start_point(start):
        return start.pose.position.x, start.pose.position.y

    def pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def marker(self, ns, id, type, scale, color):
        marker = Marker(ns=ns, id=id, type=type, action=Marker.ADD)
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        return marker

    def publish_rooms(self):
        grid, labels = self.planner.grid, self.planner.labels
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for room, (area, (x, y)) in self.planner.rooms.items():
            color = PALETTE[(room - 1) % len(PALETTE)] + (0.35,)
            cells = self.marker('rooms', room, Marker.CUBE_LIST, (grid.resolution, grid.resolution, 0.01), color)
            rows, cols = (labels == room).nonzero()
            cells.points = [Point(x=px, y=py) for px, py in (grid.to_point(r, c) for r, c in zip(rows, cols))]
            label = self.marker('room_ids', room, Marker.TEXT_VIEW_FACING, (0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))
            label.pose.position = Point(x=x, y=y, z=0.3)
            label.text = str(room)
            markers.markers += [cells, label]
        self.rooms_pub.publish(markers)

    def publish_viewpoints(self, start, poses):
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        tour = self.marker('tour', 0, Marker.LINE_STRIP, (0.02, 0.0, 0.0), (0.2, 0.4, 1.0, 0.8))
        tour.points = [start.pose.position] + [Point(x=x, y=y) for x, y, _ in poses]
        fovs = self.marker('fields_of_view', 0, Marker.LINE_LIST, (0.01, 0.0, 0.0), (1.0, 0.6, 0.0, 0.6))
        markers.markers += [tour, fovs]
        for i, (x, y, yaw) in enumerate(poses):
            arrow = self.marker('viewpoints', i, Marker.ARROW, (0.3, 0.04, 0.04), (1.0, 0.1, 0.1, 1.0))
            arrow.pose = self.pose(x, y, yaw).pose
            markers.markers.append(arrow)
            cos, sin = math.cos(yaw), math.sin(yaw)
            corners = [Point(x=x + fx * cos - fy * sin, y=y + fx * sin + fy * cos) for fx, fy in self.field_of_view]
            for a, b in zip(corners, corners[1:] + corners[:1]):
                fovs.points += [a, b]
        self.viewpoints_pub.publish(markers)


def main():
    rclpy.init()
    node = ExplorationPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
