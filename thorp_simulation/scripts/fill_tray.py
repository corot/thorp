#!/usr/bin/env python3
"""
Fill Thorp's tray with objects, both in Gazebo and in the planning scene, as if placed there: one per slot, in the
order the slots are filled, leaving the last rows free. Hidden objects go only to Gazebo, on the first free slots, so
placing there stalls against an object the planning scene doesn't know about; they are heavy, so the arm can't push
them away.
Usage:
    ros2 run thorp_simulation fill_tray.py [--model tower] [--free-rows 1] [--hidden 0]
Author:
    Jorge Santos
"""
import argparse
import json
import os
import re
import struct
import sys
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, ObjectColor, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from ros_gz_interfaces.srv import SpawnEntity
from shape_msgs.msg import Mesh, MeshTriangle

from thorp_toolkit.common import init
from thorp_toolkit.geometry import TF2, create_3d_pose, to_transform, transform_pose

ROBOT_MODEL = 'thorp'
# Spawned exactly on the tray surface, objects would start intersecting it
DROP_HEIGHT = 0.003
HIDDEN_MASS_FACTOR = 100.0


def scale_mass(sdf, factor):
    """ The model with its mass and inertia scaled """
    return re.sub(r'<(mass|ixx|iyy|izz)>([^<]+)</\1>', lambda m: f'<{m[1]}>{float(m[2]) * factor}</{m[1]}>', sdf)


def load_stl(path):
    """ Vertices and triangles of an STL mesh, binary or ASCII """
    with open(path, 'rb') as f:
        data = f.read()
    points = []
    if data[:5] == b'solid' and b'facet' in data[:300]:
        for line in data.decode(errors='ignore').splitlines():
            if line.strip().startswith('vertex'):
                points.append(tuple(float(v) for v in line.split()[1:4]))
    else:
        for i in range(struct.unpack('<I', data[80:84])[0]):
            values = struct.unpack('<12f', data[84 + i * 50:132 + i * 50])
            points += [values[3:6], values[6:9], values[9:12]]
    return points, [points[i:i + 3] for i in range(0, len(points), 3)]


class TrayFiller:
    def __init__(self, node, model, world):
        self.node = node
        self.model = model
        self.world = world
        models_path = os.path.join(get_package_share_directory('thorp_simulation'), 'worlds', 'gazebo', 'models')
        with open(os.path.join(models_path, model, 'model.sdf')) as f:
            self.sdf = f.read()
        # the object models' meshes are thorp_perception's object templates
        points, triangles = load_stl(os.path.join(get_package_share_directory('thorp_perception'), 'meshes',
                                                  model + '.stl'))
        self.size = [max(p[k] for p in points) - min(p[k] for p in points) for k in range(3)]
        self.mesh = Mesh(vertices=[Point(x=p[0], y=p[1], z=p[2]) for p in points],
                         triangles=[MeshTriangle(vertex_indices=[3 * i, 3 * i + 1, 3 * i + 2])
                                    for i in range(len(triangles))])

        tray_config = os.path.join(get_package_share_directory('thorp_description'), 'config', 'tray.yaml')
        with open(tray_config) as f:
            tray = yaml.safe_load(f)['/**']['ros__parameters']['tray']
        self.tray_link = tray['link']
        self.slot = tray['slot']
        self.slots_x = round(tray['side_x'] / self.slot + 0.1)
        self.slots_y = round(tray['side_y'] / self.slot + 0.1)

        self.spawn_client = node.create_client(SpawnEntity, f'/world/{world}/create')
        self.get_scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.robot_pose = None
        self.robot_pose_received = threading.Event()
        self.gz_node = GzNode()
        self.gz_node.subscribe(Pose_V, f'/world/{world}/pose/info', self.poses_cb)

    def poses_cb(self, msg):
        for pose in msg.pose:
            if pose.name == ROBOT_MODEL:
                robot_pose = PoseStamped()
                robot_pose.header.frame_id = 'world'
                robot_pose.pose.position.x, robot_pose.pose.position.y = pose.position.x, pose.position.y
                robot_pose.pose.position.z = pose.position.z
                robot_pose.pose.orientation.x, robot_pose.pose.orientation.y = pose.orientation.x, pose.orientation.y
                robot_pose.pose.orientation.z, robot_pose.pose.orientation.w = pose.orientation.z, pose.orientation.w
                self.robot_pose = robot_pose
                self.robot_pose_received.set()

    def slots(self):
        """ Slot poses on the tray frame, in filling order: by rows along y, and along x within each row """
        return [create_3d_pose((i - self.slots_x / 2.0 + 0.5) * self.slot, (j - self.slots_y / 2.0 + 0.5) * self.slot,
                               0.0, 0.0, 0.0, 0.0, self.tray_link)
                for j in range(self.slots_y) for i in range(self.slots_x)]

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'Service {client.srv_name} not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.result() is None:
            raise RuntimeError(f'Calling {client.srv_name} failed')
        return future.result()

    def spawn(self, name, slot, sdf):
        """ Spawn the model on a slot, on Gazebo's world frame, through the robot's pose there """
        on_base = TF2().transform_pose(slot, slot.header.frame_id, 'base_footprint')
        on_base.pose.position.z += DROP_HEIGHT
        request = SpawnEntity.Request()
        request.entity_factory.name = name
        request.entity_factory.sdf = sdf
        request.entity_factory.pose = transform_pose(on_base, to_transform(self.robot_pose, ROBOT_MODEL)).pose
        if not self.call(self.spawn_client, request).success:
            raise RuntimeError(f'Spawning {name} failed')

    def get_scene(self, components):
        request = GetPlanningScene.Request()
        request.components.components = components
        return self.call(self.get_scene_client, request).scene

    def free_names(self, count):
        """ Names as perception gives them, the type plus the first free indexes """
        scene = self.get_scene(PlanningSceneComponents.WORLD_OBJECT_NAMES |
                               PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        taken = {o.id for o in scene.world.collision_objects}
        taken |= {o.object.id for o in scene.robot_state.attached_collision_objects}
        names = []
        index = 1
        while len(names) < count:
            if f'{self.model} {index}' not in taken:
                names.append(f'{self.model} {index}')
            index += 1
        return names

    def add_to_scene(self, names, slots):
        """ Add the objects as perception would, carrying the model's mesh, resting on the slots """
        scene = PlanningScene(is_diff=True)
        for name, slot in zip(names, slots):
            pose = create_3d_pose(slot.pose.position.x, slot.pose.position.y, self.size[2] / 2.0, 0.0, 0.0, 0.0)
            mesh_pose = create_3d_pose(0.0, 0.0, -self.size[2] / 2.0, 0.0, 0.0, 0.0)  # meshes' origin is at their base
            obj = CollisionObject(id=name, operation=CollisionObject.ADD, meshes=[self.mesh], mesh_poses=[mesh_pose])
            obj.header.frame_id = self.tray_link
            obj.pose = pose
            obj.type.db = json.dumps({'size': sorted(self.size[:2], reverse=True) + [self.size[2]]})
            scene.world.collision_objects.append(obj)
            scene.object_colors.append(ObjectColor(id=name))
            scene.object_colors[-1].color.r, scene.object_colors[-1].color.g = 0.5, 0.8
            scene.object_colors[-1].color.b, scene.object_colors[-1].color.a = 1.0, 1.0
        scene.allowed_collision_matrix = self.allow_tray_contacts(names)
        if not self.call(self.apply_scene_client, ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError('Adding the objects to the planning scene failed')

    def allow_tray_contacts(self, names):
        """
        The current allowed collision matrix, plus the objects resting on the tray, as placing leaves them: a scene
        diff replaces the whole matrix
        """
        acm = self.get_scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
        new_entries = names + ([self.tray_link] if self.tray_link not in acm.entry_names else [])
        for entry in acm.entry_values:
            entry.enabled += [False] * len(new_entries)
        acm.entry_names += new_entries
        acm.entry_values += [AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)) for _ in new_entries]
        tray = acm.entry_names.index(self.tray_link)
        for name in names:
            obj = acm.entry_names.index(name)
            acm.entry_values[obj].enabled[tray] = acm.entry_values[tray].enabled[obj] = True
        return acm

    def fill(self, free_rows, hidden):
        if not self.robot_pose_received.wait(timeout=10.0):
            raise RuntimeError(f'No pose for {ROBOT_MODEL} from Gazebo')
        slots = self.slots()
        filled = slots[:len(slots) - free_rows * self.slots_x]
        hidden_slots = slots[len(filled):len(filled) + hidden]
        names = self.free_names(len(filled))
        for k, slot in enumerate(filled):
            self.spawn(f'tray_{self.model}_{k}', slot, self.sdf)
        for k, slot in enumerate(hidden_slots):
            self.spawn(f'tray_hidden_{self.model}_{k}', slot, scale_mass(self.sdf, HIDDEN_MASS_FACTOR))
        self.add_to_scene(names, filled)
        self.node.get_logger().info(f'{len(filled)} {self.model} objects on the tray, plus {len(hidden_slots)} '
                                    f'only in Gazebo')


def main():
    parser = argparse.ArgumentParser(description='Fill the tray with objects, in Gazebo and in the planning scene')
    parser.add_argument('--model', default='tower', help='Gazebo model to fill the tray with')
    parser.add_argument('--free-rows', type=int, default=1, help='rows of slots to leave free, the last to fill')
    parser.add_argument('--hidden', type=int, default=0, help='objects only in Gazebo, on the first free slots')
    parser.add_argument('--world', default='default', help='Gazebo world name')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('fill_tray')
    init(node)
    try:
        TrayFiller(node, args.model, args.world).fill(args.free_rows, args.hidden)
    except RuntimeError as err:
        node.get_logger().error(str(err))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
