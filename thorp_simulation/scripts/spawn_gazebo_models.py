#!/usr/bin/env python3

"""
Populate the Gazebo world with tables and objects, or cats:
 - playground_fixed: a table with a sample of objects at reachable locations, in front of the robot
 - playground_cubes: a table with cubes at reachable locations, ready to stack
 - playground_rows: a table with 5 rows of 8 cubes
 - playground_random: a random table with random objects, in front of the robot
 - fun_house_objects: random tables with random objects, in open spaces of the map (any map, despite the
   name); needs navigation running
 - cats: cats for the hunter, in open spaces of the map, and the rocket for the cannon; needs navigation running
Usage:
    ros2 run thorp_simulation spawn_gazebo_models.py <mode> [-l]
    -l: place random tables or cats at preferred locations
Author:
    Jorge Santos
"""

import os
import random
import sys

from math import pi, copysign, cos, sin, sqrt

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from ros_gz_interfaces.srv import SpawnEntity

from thorp_toolkit.common import init
from thorp_toolkit.geometry import TF2, distance_2d, create_2d_pose, create_3d_pose, pose2d2str, yaw

# name: surface name
# size: x, y dimensions
# objs: number of tabletop objects to spawn
# dist: distributions models (one of 'uniform', 'diagonal', 'xor', '+/+')
# count: number of surfaces of this type to spawn
surfaces = [{'name': 'doll_table',
             'size': (0.45, 0.45),
             'objs': 4,
             'dist': 'uniform',
             'count': 2},
            {'name': 'lack_table',
             'size': (0.55, 0.55),
             'objs': 6,
             'dist': 'uniform',
             'count': 2},
            {'name': 'lack_table_x15',
             'size': (0.825, 0.55),
             'objs': 10,
             'dist': 'uniform',
             'count': 1}
            ]

objects = ['wood_cube_2_5cm',
           'tower',
           'cube',
           'square',
           'rectangle',
           'triangle',
           'pentagon',
           'circle',
           'star',
           'cross',
           'clover']

# a sample of objects mostly at reachable locations; the cross, circle and pentagon face the arm docked at the
# table, to be grasped across their wide side
PLAYGROUND_OBJS = [('square',    'square',    (-0.22,  0.15,  0.5, 0.0, 0.0, 0.4)),
                   ('cross',     'cross',     (-0.06,  0.15,  0.5, 0.0, 0.0, 1.95)),
                   ('circle',    'circle',    (-0.06,  0.04,  0.5, 0.0, 0.0, 1.61)),
                   ('cube',      'cube',      (0.01,   0.1,   0.5, 0.0, 0.0, 0.85)),
                   ('triangle',  'triangle',  (-0.13,  0.1,   0.5, 0.0, 0.0, 0.1)),
                   ('star',      'star',      (-0.17,  0.017, 0.5, 0.0, 0.0, 0.2)),
                   ('clover',    'clover',    (-0.11, -0.037, 0.5, 0.0, 0.0, 1.4)),
                   ('pentagon',  'pentagon',  (-0.12, -0.10,  0.5, 0.0, 0.0, 1.08)),
                   ('rectangle', 'rectangle', (-0.16, -0.16,  0.5, 0.0, 0.0, 1.1))]

# cubes at reachable locations, ready to stack
PLAYGROUND_CUBES = [('cube 1', 'cube', (-0.14, -0.16, 0.5, 0.0, 0.0, 1.1)),
                    ('cube 2', 'cube', (-0.11, -0.10, 0.5, 0.0, 0.0, 0.15)),
                    ('cube 3', 'cube', (-0.15,  0.02, 0.5, 0.0, 0.0, 0.2)),
                    ('cube 4', 'cube', (-0.11,  0.10, 0.5, 0.0, 0.0, 0.85)),
                    ('cube 5', 'cube', (-0.12,  0.15, 0.5, 0.0, 0.0, 0.4))]

# 5 rows of 8 cubes tightly spaced; tailored for lack table
N_ROWS_OF_CUBES = [('cube ' + str(i), 'cube',
                    (((i // 8) - 2) / 10, ((i % 8) - 4) / 18.0, 0.45, 0.0, 0.0, 0.0)) for i in range(40)]

cats = [{'name': 'cat_black', 'count': 3},
        {'name': 'cat_orange', 'count': 3}]

SURFS_MIN_DIST = 1.5
OBJS_MIN_DIST = 0.08
CATS_MIN_DIST = 3.0
CATS_CLEARANCE = 0.3

PREFERRED_LOCATIONS = [(12.5, 7.5),
                       (8.4, 9.6),
                       (4.6, 9.8),
                       (5.8, 6.5)]


class ModelsSpawner(Node):
    def __init__(self):
        super().__init__('spawn_gazebo_models')
        init(self)
        self.spawned = {o: 0 for o in objects}
        models_path = os.path.join(get_package_share_directory('thorp_simulation'), 'worlds', 'gazebo', 'models')
        self.models = {}
        for model in objects + [s['name'] for s in surfaces] + [c['name'] for c in cats] + ['rocket']:
            with open(os.path.join(models_path, model, 'model.sdf')) as f:
                self.models[model] = f.read()
        self.spawn_client = self.create_client(SpawnEntity, '/world/default/create')
        if not self.spawn_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("Service '/world/default/create' not available")
        self.costmap = None

    def spawn_model(self, name, model, pose, surface_pose=None):
        """ Spawn model on the given world pose, or relative to a surface's pose if provided """
        if surface_pose:
            # Gazebo can't spawn relative to a model created on the same step, so we compose the poses here
            surface_yaw = yaw(surface_pose)
            pose = create_3d_pose(surface_pose.position.x + cos(surface_yaw) * pose.position.x
                                  - sin(surface_yaw) * pose.position.y,
                                  surface_pose.position.y + sin(surface_yaw) * pose.position.x
                                  + cos(surface_yaw) * pose.position.y,
                                  pose.position.z, 0.0, 0.0, surface_yaw + yaw(pose))
        request = SpawnEntity.Request()
        request.entity_factory.name = name
        request.entity_factory.sdf = self.models[model]
        request.entity_factory.pose = pose
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f"Spawn model {name} failed")
        return future.result().success

    def wait_for_map(self, topic):
        """ Get a map or costmap, published as a latched occupancy grid """
        grids = []
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.create_subscription(OccupancyGrid, topic, grids.append, qos)
        while rclpy.ok() and not grids:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.destroy_subscription(sub)
        return grids[0]

    def close_to_obstacle(self, x, y, clearance):
        """ Check if the location is closer than clearance to any non-free global costmap cell """
        info = self.costmap.info
        cells = int(clearance / info.resolution)
        cx = int((x - info.origin.position.x) / info.resolution)
        cy = int((y - info.origin.position.y) / info.resolution)
        for i in range(cx - cells, cx + cells + 1):
            for j in range(cy - cells, cy + cells + 1):
                if (i - cx) ** 2 + (j - cy) ** 2 > cells ** 2:
                    continue
                # cells outside the costmap, unknown or with any cost are all obstacles
                if not (0 <= i < info.width and 0 <= j < info.height) or self.costmap.data[j * info.width + i] != 0:
                    return True
        return False

    def spawn_objects(self, surf, surf_index, surf_pose, preferred_obj=None):
        added_poses = [create_3d_pose(0, 0, 0, 0, 0, 0)]  # fake pose to avoid the (non-reachable) surface's center
        obj_index = 0
        margin = 0.1  # no obstacles at table margins
        reachable = 0.2  # no obstacles beyond arm reach

        # Compute the unreachable inner area
        inner_min_x = -surf['size'][0] / 2.0 + reachable
        inner_max_x = +surf['size'][0] / 2.0 - reachable
        inner_min_y = -surf['size'][1] / 2.0 + reachable
        inner_max_y = +surf['size'][1] / 2.0 - reachable
        while obj_index < surf['objs'] and rclpy.ok():
            obj_name = preferred_obj or random.choice(objects)

            # even distribution
            x = random.uniform((-surf['size'][0] + margin) / 2.0, (+surf['size'][0] - margin) / 2.0)
            y = random.uniform((-surf['size'][1] + margin) / 2.0, (+surf['size'][1] - margin) / 2.0)

            # check if the generated position is within the unreachable inner area
            if inner_min_x <= x <= inner_max_x and inner_min_y <= y <= inner_max_y:
                continue

            if surf['dist'] == 'diagonal':
                # half-surface by diagonal
                if x + y < 0:
                    x = -x
                    y = -y
            elif surf['dist'] == 'xor':
                # +x/-y or -x/+y quadrants
                if x * y < 0:
                    x = copysign(x, y)
            elif surf['dist'] == '+/+':
                # only +x/+y quadrant
                x = abs(x)
                y = abs(y)
            elif surf['dist'] != 'uniform':
                raise ValueError("Unknown distribution " + surf['dist'])

            z = 0.5
            pose = create_3d_pose(x, y, z, 0, 0, random.uniform(-pi, +pi))
            # we check that the distance to all previously added objects is below a threshold to space the objects
            if any(distance_2d(pose, p) < OBJS_MIN_DIST for p in added_poses):
                continue
            added_poses.append(pose)
            model_name = '_'.join([surf['name'], str(surf_index), obj_name, str(obj_index)])
            if self.spawn_model(model_name, obj_name, pose, surf_pose):
                self.spawned[obj_name] += 1
            obj_index += 1

    def spawn_surfaces(self, use_preferred_locs=False):
        # random locations within the map, away from the robot and in open space
        grid = self.wait_for_map('map')
        min_x = grid.info.origin.position.x
        min_y = grid.info.origin.position.y
        max_x = min_x + grid.info.width * grid.info.resolution
        max_y = min_y + grid.info.height * grid.info.resolution
        self.costmap = self.wait_for_map('global_costmap/costmap')
        robot_pose = TF2().transform_pose(None, 'base_footprint', 'map')

        added_poses = []  # to check that tables are at least SURFS_MIN_DIST apart from each other
        for surf in surfaces:
            surf_index = 0
            # clearance = surface diagonal / 2 + some extra margin
            clearance = sqrt((surf['size'][0] / 2.0) ** 2 + (surf['size'][1] / 2.0) ** 2) + 0.1
            while surf_index < surf['count'] and rclpy.ok():
                if use_preferred_locs:
                    x, y = random.choice(PREFERRED_LOCATIONS)
                else:
                    x = random.uniform(min_x, max_x)
                    y = random.uniform(min_y, max_y)
                theta = random.uniform(-pi, +pi)
                pose = create_2d_pose(x, y, theta)
                # we check that the distance to all previously added surfaces is below a threshold to space them
                if any(distance_2d(pose, p) < SURFS_MIN_DIST for p in added_poses):
                    continue

                # check also that the surface is not too close to the robot
                if distance_2d(pose, robot_pose) < SURFS_MIN_DIST:
                    continue

                # and in open space
                if self.close_to_obstacle(x, y, clearance):
                    continue

                added_poses.append(pose)
                model = surf['name']
                model_name = model + '_' + str(surf_index + 10)  # allow for some objects added by hand
                if self.spawn_model(model_name, model, pose):
                    self.get_logger().info(f"Spawned {model_name} at {pose2d2str(pose)}")
                    # populate this surface with some objects
                    self.spawn_objects(surf, surf_index + 10, pose)
                surf_index += 1

    def spawn_cats(self, use_preferred_locs=False):
        # random locations within the map, away from each other and from the robot, and in open space
        grid = self.wait_for_map('map')
        min_x = grid.info.origin.position.x
        min_y = grid.info.origin.position.y
        max_x = min_x + grid.info.width * grid.info.resolution
        max_y = min_y + grid.info.height * grid.info.resolution
        self.costmap = self.wait_for_map('global_costmap/costmap')
        robot_pose = TF2().transform_pose(None, 'base_footprint', 'map')

        added_poses = []
        for cat in cats:
            cat_index = 0
            while cat_index < cat['count'] and rclpy.ok():
                if use_preferred_locs:
                    x, y = random.choice(PREFERRED_LOCATIONS)
                else:
                    x = random.uniform(min_x, max_x)
                    y = random.uniform(min_y, max_y)
                pose = create_2d_pose(x, y, random.uniform(-pi, +pi))
                if any(distance_2d(pose, p) < CATS_MIN_DIST for p in added_poses + [robot_pose]):
                    continue
                if self.close_to_obstacle(x, y, CATS_CLEARANCE):
                    continue

                added_poses.append(pose)
                model_name = cat['name'] + '_' + str(cat_index)
                if self.spawn_model(model_name, cat['name'], pose):
                    self.get_logger().info(f"Spawned {model_name} at {pose2d2str(pose)}")
                cat_index += 1

    def spawn(self, mode, use_preferred_locs):
        self.get_logger().info(f"Spawning {mode} in {'preferred' if use_preferred_locs else 'random'} locations")
        if mode == 'fun_house_objects':
            self.spawn_surfaces(use_preferred_locs)
            self.get_logger().info("Spawned objects:\n  " +
                                   '\n  '.join(f'{k}: {v}' for k, v in self.spawned.items()))
        elif mode == 'cats':
            self.spawn_cats(use_preferred_locs)
            # the cannon places the rocket at its muzzle when firing; until then, out of sight
            self.spawn_model('rocket', 'rocket', create_3d_pose(-10.0, -10.0, 0.03, 0.0, 0.0, 0.0))
        elif mode == 'playground_random':  # random objects over a random table
            surface = random.choice(surfaces)
            surf_name = surface['name']
            surf_pose = create_2d_pose(0.45, 0.0, pi / 2.0)
            self.spawn_model(surf_name + '_0', surf_name, surf_pose)
            self.spawn_objects(surface, 0, surf_pose)
        elif mode in ('playground_fixed', 'playground_cubes', 'playground_rows'):
            objs = {'playground_fixed': PLAYGROUND_OBJS,  # a sample of objects mostly at reachable locations
                    'playground_cubes': PLAYGROUND_CUBES,  # cubes at reachable locations, ready to stack
                    'playground_rows': N_ROWS_OF_CUBES}[mode]  # rows of cubes
            surf_pose = create_2d_pose(0.45, 0.0, 0.0)
            self.spawn_model('lack_table', 'lack_table', surf_pose)
            for obj in objs:
                self.spawn_model(obj[0], obj[1], create_3d_pose(*obj[2]), surf_pose)
        else:
            raise ValueError(f"Unknown mode {mode}")


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    if len(args) < 2:
        print("Usage: spawn_gazebo_models.py playground_fixed | playground_cubes | playground_rows | "
              "playground_random | fun_house_objects | cats [-l]")
        sys.exit(-1)
    rclpy.init()
    node = ModelsSpawner()
    random.seed()
    try:
        node.spawn(args[1], '-l' in args[2:])
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
