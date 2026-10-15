#!/usr/bin/env python3
"""
thorp_world's node: what Thorp knows about the world, served to detections, skills and the agent.

Services, under the node's name:
- observe (thorp_msgs/ObserveWorld): detections report what they saw, and get the ids of what they saw
- update (thorp_msgs/UpdateWorld): skills report their effects
- query (thorp_msgs/QueryWorld): what is known, with the relations that depend on where the robot is
- reach (thorp_msgs/QueryReach): which objects on a table the arm would reach with the robot elsewhere
- save, load (std_srvs/Trigger): to and from world_file, also loaded on start if it exists; the parameter can change
  at any time

Topics, latched: state (thorp_msgs/WorldState) and markers (visualization_msgs/MarkerArray), whenever it changes.
"""

import os

import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from thorp_msgs.msg import ObjectsInReach, ObjectToPickup, WorldState
from thorp_msgs.srv import ObserveWorld, QueryReach, QueryWorld, UpdateWorld
from thorp_toolkit.visualization import Visualization

from thorp_world import OBJECT, OBSERVED, OK, TABLE, World
from thorp_world.conversions import to_msg, to_observation, to_pose, to_seconds


class WorldNode(Node):
    def __init__(self):
        super().__init__('world')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').value
        self.arm_frame = self.declare_parameter('arm_frame', 'arm_base_link').value
        self.max_arm_reach = self.declare_parameter('max_arm_reach', 0.3).value
        self.tf_timeout = Duration(seconds=self.declare_parameter('tf_timeout', 0.5).value)
        self.declare_parameter('world_file', '')
        self.world = World(object_tolerance=self.declare_parameter('object_tolerance', 0.05).value,
                           table_tolerance=self.declare_parameter('table_tolerance', 0.3).value,
                           table_area_ratio=self.declare_parameter('table_area_ratio', 0.8).value,
                           at_table_distance=self.declare_parameter('at_table_distance', 0.3).value)

        # its own thread, as lookups at a detection's stamp wait for transforms from within service callbacks
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(WorldState, '~/state', latched)
        self.markers_pub = self.create_publisher(MarkerArray, '~/markers', latched)
        self.create_service(ObserveWorld, '~/observe', self.observe)
        self.create_service(UpdateWorld, '~/update', self.update)
        self.create_service(QueryWorld, '~/query', self.query)
        self.create_service(QueryReach, '~/reach', self.reach)
        self.create_service(Trigger, '~/save', self.save)
        self.create_service(Trigger, '~/load', self.load)

        world_file = self.world_file()
        if world_file and os.path.exists(world_file):
            self.world.load(world_file)
            self.get_logger().info(f"Loaded {len(self.world.entities())} entities from '{world_file}'")
        self.publish()

    def observe(self, request, response):
        try:
            # every transform before any change, so a failed observation leaves the world as it was
            tables = [to_observation(t, self.to_map(t.header, t.pose)) for t in request.tables]
            surface = objects = None
            if request.surface.id:
                surface = to_observation(request.surface, self.to_map(request.surface.header, request.surface.pose))
                objects = [to_observation(o, self.to_map(o.header, o.pose)) for o in request.objects]
        except (TransformException, ValueError) as e:
            response.success, response.message = False, f'cannot take the observation to {self.map_frame}: {e}'
            return response

        if tables:
            response.table_ids = self.world.observe_tables(tables, self.stamp_of(request.tables[0].header))
        if surface is not None:
            try:
                robot = to_pose(self.frame_on_map(self.robot_frame, request.surface.header.stamp))
            except TransformException:
                robot = None  # objects not seen then can't be told missing
            response.surface_id, response.object_ids = \
                self.world.observe_objects(surface, objects, self.stamp_of(request.surface.header), robot)
        response.success = True
        self.publish()
        return response

    def update(self, request, response):
        now = to_seconds(self.get_clock().now().to_msg())
        try:
            if request.effect == UpdateWorld.Request.PICKED:
                self.world.mark_picked(request.object_id, now)
            elif request.effect == UpdateWorld.Request.PLACED:
                pose = to_pose(self.to_map(request.pose.header, request.pose.pose))
                self.world.mark_placed(request.object_id, now, request.table_id, pose)
            elif request.effect == UpdateWorld.Request.STOWED:
                self.world.mark_stowed(request.object_id, now, request.slot if request.slot >= 0 else None)
            elif request.effect == UpdateWorld.Request.DISTURBED:
                self.world.mark_disturbed(request.object_id, now)
            elif request.effect == UpdateWorld.Request.DOCKED:
                self.world.dock(request.table_id)
            elif request.effect == UpdateWorld.Request.UNDOCKED:
                self.world.undock()
            elif request.effect == UpdateWorld.Request.LOST:
                self.world.mark_lost(request.object_id, now)
            elif request.effect == UpdateWorld.Request.INSPECTED:
                self.world.mark_inspected(request.table_id, now)
            else:
                raise ValueError(f'unknown effect {request.effect}')
        except (KeyError, ValueError, TransformException) as e:
            response.success, response.message = False, str(e).strip('"')
            return response
        response.success = True
        self.publish()
        return response

    def query(self, request, response):
        entities = [e for e in self.world.entities() if
                    (not request.ids or e.id in request.ids) and (not request.kind or e.kind == request.kind) and
                    (not request.support or e.support == request.support) and
                    (not request.status or e.status == request.status)]
        response.entities = [to_msg(e, self.map_frame) for e in entities]
        response.reachable = [False] * len(entities)
        response.success = True
        try:
            robot = to_pose(self.frame_on_map(self.robot_frame))
            arm = to_pose(self.frame_on_map(self.arm_frame))
        except TransformException as e:
            response.message = f'no robot pose, so no relations: {e}'
            return response
        response.reachable = [e.kind == OBJECT and self.world.within_reach(e.id, arm, self.max_arm_reach)
                              for e in entities]
        response.at_table = self.world.table_at(robot) or ''
        return response

    def reach(self, request, response):
        response.reach = self.max_arm_reach - request.margin
        try:
            arm_on_robot = self.tf_buffer.lookup_transform(self.robot_frame, self.arm_frame, Time(),
                                                           timeout=self.tf_timeout)
            robot_poses = [self.to_map(p.header, p.pose) for p in request.robot_poses]
        except TransformException as e:
            response.success, response.message = False, f'cannot put the arm where the robot would be: {e}'
            return response
        # the arm's base on the robot, to put where the robot would be
        offset, arm = arm_on_robot.transform, PoseStamped()
        arm.pose.position.x, arm.pose.position.y, arm.pose.position.z = \
            offset.translation.x, offset.translation.y, offset.translation.z
        arm.pose.orientation = offset.rotation
        try:
            for robot_pose in robot_poses:
                robot = TransformStamped(header=robot_pose.header, child_frame_id=self.robot_frame)
                position = robot_pose.pose.position
                robot.transform.translation.x, robot.transform.translation.y = position.x, position.y
                robot.transform.translation.z = position.z
                robot.transform.rotation = robot_pose.pose.orientation
                arm_pose = do_transform_pose_stamped(arm, robot)
                in_reach = ObjectsInReach(robot_pose=robot_pose, arm_pose=arm_pose)
                for object_id, distance in self.world.reachable_from(request.table_id, to_pose(arm_pose),
                                                                     response.reach):
                    in_reach.objects.append(ObjectToPickup(name=object_id, distance=distance,
                                                           pose=to_msg(self.world.get(object_id), self.map_frame).pose))
                response.in_reach.append(in_reach)
        except KeyError as e:
            response.success, response.message = False, str(e).strip('"')
            response.in_reach = []
            return response
        response.success = True
        return response

    def save(self, request, response):
        world_file = self.world_file()
        if not world_file:
            response.success, response.message = False, 'no world_file parameter to save to'
            return response
        try:
            self.world.save(world_file)
        except OSError as e:
            response.success, response.message = False, f"cannot save to '{world_file}': {e}"
            return response
        response.success, response.message = True, f'saved to {world_file}'
        return response

    def load(self, request, response):
        world_file = self.world_file()
        if not world_file or not os.path.exists(world_file):
            response.success, response.message = False, f"no world file '{world_file}' to load"
            return response
        try:
            self.world.load(world_file)
        except (OSError, ValueError, yaml.YAMLError) as e:
            response.success, response.message = False, f"cannot load '{world_file}': {e}"
            return response
        response.success, response.message = True, f'loaded from {world_file}'
        self.publish()
        return response

    def world_file(self):
        return os.path.expanduser(self.get_parameter('world_file').value)

    def to_map(self, header, pose):
        """A pose on the map frame, as it was when stamped; the latest transform if not stamped."""
        transform = self.tf_buffer.lookup_transform(self.map_frame, header.frame_id, Time.from_msg(header.stamp),
                                                    timeout=self.tf_timeout)
        return do_transform_pose_stamped(PoseStamped(header=header, pose=pose), transform)

    def frame_on_map(self, frame, stamp=None):
        """Where a frame is on the map frame, at stamp; now if not given."""
        pose = PoseStamped()
        pose.header.frame_id = frame
        if stamp is not None:
            pose.header.stamp = stamp
        pose.pose.orientation.w = 1.0
        return self.to_map(pose.header, pose.pose)

    def stamp_of(self, header):
        stamp = to_seconds(header.stamp)
        return stamp if stamp > 0.0 else to_seconds(self.get_clock().now().to_msg())

    def publish(self):
        entities = self.world.entities()
        self.state_pub.publish(WorldState(entities=[to_msg(e, self.map_frame) for e in entities],
                                          docked_at=self.world.docked_at or ''))
        markers = [Marker(action=Marker.DELETEALL)]
        for entity in entities:
            if entity.pose is None:
                continue
            msg = to_msg(entity, self.map_frame)
            msg.pose.header.stamp = TimeMsg()  # the latest transform, whatever frame RViz shows them on
            color = self.color_of(entity)
            box = Visualization.create_geometry_primitive_marker(msg.pose, list(entity.size), color, Marker.CUBE,
                                                                 entity.kind + 's')
            label_pose = PoseStamped(header=msg.pose.header, pose=msg.pose.pose)
            label_pose.pose.position.z += entity.size[2] / 2.0 + 0.05
            label = Visualization.create_text_marker(label_pose, entity.id, 0.04, color, entity.kind + ' labels')
            markers.extend([box, label])
        for index, marker in enumerate(markers):
            marker.id = index
        self.markers_pub.publish(MarkerArray(markers=markers))

    @staticmethod
    def color_of(entity):
        """Faded for what a skill believes, gray for what's missing or was disturbed."""
        if entity.status != OK:
            return Visualization.named_color('gray', 0.5)
        alpha = 0.4 if entity.kind == TABLE else 0.9 if entity.provenance == OBSERVED else 0.4
        try:
            return Visualization.named_color(entity.color, alpha)
        except ValueError:
            return Visualization.named_color('light gray', alpha)


def main():
    rclpy.init()
    node = WorldNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
