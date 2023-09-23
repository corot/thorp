#!/usr/bin/env python

"""
Populate gazebo world with randomly spawn models:
 - cats for hunter
 - tables and objects for gatherer
 - cubes at known locations on playground world
 - objects at known locations on playground world
Author:
    Jorge Santos
"""

import random
import sys
import rospy
import rospkg

from math import pi, copysign, sqrt
from itertools import product

from nav_msgs.msg import MapMetaData
from mbf_msgs.srv import CheckPose, CheckPoseRequest
from gazebo_msgs.srv import SpawnModel, DeleteModel

from thorp_toolkit.geometry import TF2, distance_2d, create_2d_pose, create_3d_pose, pose2d2str


surfaces = [{'name': 'doll_table',
             'size': (0.45, 0.45),
             'objs': 4,
             'dist': 'uniform',  # different distributions: 'uniform', 'diagonal', 'xor', '+/+'
             'count': 2},
            {'name': 'lack_table',
             'size': (0.55, 0.55),
             'objs': 6,
             'dist': 'uniform',  # different distributions: 'uniform', 'diagonal', 'xor', '+/+'
             'count': 2},
            {'name': 'lack_table_x15',
             'size': (0.825, 0.55),
             'objs': 10,
             'dist': 'uniform',  # different distributions: 'uniform', 'diagonal', 'xor', '+/+'
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
spawned = {o: 0 for o in objects}

cats = [{'name': 'cat_black', 'count': 2},
        {'name': 'cat_orange', 'count': 2}]
models = {}

# a sample of objects mostly at reachable locations
PLAYGROUND_OBJS = [('star', 'star', (0.28, 0.017, 0.5, 0.0, 0.0, 0.2)),
                   ('clover', 'clover', (0.36, -0.037, 0.5, 0.0, 0.0, 1.4)),
                   ('square', 'square', (0.29, -0.16, 0.5, 0.0, 0.0, 1.1)),
                   ('triangle', 'triangle', (0.32, 0.1, 0.5, 0.0, 0.0, 0.1)),
                   ('rectangle', 'rectangle', (0.27, 0.15, 0.5, 0.0, 0.0, 0.4)),
                   ('cross', 'cross', (0.37, 0.15, 0.5, 0.0, 0.0, 0.75)),
                   ('tower', 'tower', (0.38, -0.14, 0.5, 0.0, 0.0, 0.15)),
                   ('circle', 'circle', (0.48, 0.12, 0.5, 0.0, 0.0, 1.15)),
                   ('cube', 'cube', (0.46, 0.1, 0.5, 0.0, 0.0, 0.85))]

# cubes at reachable locations, ready to stack
PLAYGROUND_CUBES = [('cube 1', 'cube', (-0.15, 0.017, 0.45, 0.0, 0.0, 0.2)),
                    ('cube 2', 'cube', (-0.11, -0.10, 0.45, 0.0, 0.0, 0.15)),
                    ('cube 3', 'cube', (-0.14, -0.16, 0.45, 0.0, 0.0, 1.1)),
                    ('cube 4', 'cube', (-0.12, 0.15, 0.45, 0.0, 0.0, 0.4)),
                    ('cube 5', 'cube', (-0.11, 0.10, 0.45, 0.0, 0.0, 0.85))]

SURFS_MIN_DIST = 1.5
OBJS_MIN_DIST = 0.08
CATS_MIN_DIST = 5.0

PREFERRED_LOCATIONS = [(12.5, 7.5),
                       (8.4, 9.6),
                       (4.6, 9.8),
                       (5.8, 6.5)]


def load_models():
    global models
    for obj in objects + [s['name'] for s in surfaces] + [c['name'] for c in cats] + ['rocket']:
        model_path = ros_pack.get_path('thorp_simulation') + '/worlds/gazebo/models/' + obj + '/model.sdf'
        models[obj] = open(model_path, 'r').read()


def spawn_model(name, model, pose, frame):
    """
    equivalent to command line:
    rosrun gazebo_ros spawn_model -sdf -database wood_cube_2_5cm -model wood_cube_2_5cm_10 -reference_frame doll_table_0::link -x 0 -y 0 -z 0.45
    """
    resp = spawn_model_client(
        model_name=name,
        model_xml=model,
        initial_pose=pose,
        reference_frame=frame
    )
    if not resp.success:
        rospy.logerr("Spawn model failed: %s", resp.status_message)
    return resp.success


def close_to_robot(pose, robot_pose, min_dist):
    # check if the pose is closer than MIN_DIST to the current robot pose
    return distance_2d(pose, robot_pose) < min_dist


def close_to_prev_pose(pose, added_poses, min_dist):
    # check if the pose is closer than MIN_DIST to any of the previous poses
    # TODO I need something less naive to add more than 10 objects, e.g. spatial hash
    for prev_pose in added_poses:
        if distance_2d(pose, prev_pose) < min_dist:
            return True
    return False


def close_to_obstacle(x, y, theta, clearance):
    if not hasattr(close_to_obstacle, 'check_srv'):
        # we need MBF's check pose service to ensure that the spawned objects are in open spaces
        close_to_obstacle.check_srv = rospy.ServiceProxy('move_base_flex/check_pose_cost', CheckPose, persistent=True)
        close_to_obstacle.check_srv.wait_for_service(10)
        close_to_obstacle.robot_radius = rospy.get_param('move_base_flex/global_costmap/robot_radius')

    # check if the location is away from any non-zero cost in the global costmap by at least the given clearance
    # (we need to subtract robot radius because check pose service assumes we want to check the footprint cost)
    resp = close_to_obstacle.check_srv(pose=create_2d_pose(x, y, theta, 'map'),
                                       safety_dist=clearance - close_to_obstacle.robot_radius,
                                       costmap=CheckPoseRequest.GLOBAL_COSTMAP)
    if resp.state > 0 or resp.cost > 0:
        return True
    return False


def spawn_objects(surf, surf_index, preferred_obj=None):
    added_poses = [create_3d_pose(0, 0, 0, 0, 0, 0)]  # fake pose to avoid the (non-reachable) surface's center
    obj_index = 0
    margin = rospy.get_param('table_margins_clearance', 0.1)  # no obstacles at table margins
    while obj_index < surf['objs'] and not rospy.is_shutdown():
        obj_name = preferred_obj or random.choice(objects)

        # even distribution
        x = random.uniform((-surf['size'][0] + margin) / 2.0, (+surf['size'][0] - margin) / 2.0)
        y = random.uniform((-surf['size'][1] + margin) / 2.0, (+surf['size'][1] - margin) / 2.0)

        if surf['dist'] == 'diagonal':
            # half surface by diagonal
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
        elif surf['dist'] == 'uniform':
            pass
        else:
            rospy.logerr("ERROR: unknown distribution " + surf['dist'])
            sys.exit(-1)

        z = 0.5
        pose = create_3d_pose(x, y, z, 0, 0, random.uniform(-pi, +pi))
        # we check that the distance to all previously added objects is below a threshold to space the objects
        if close_to_prev_pose(pose, added_poses, OBJS_MIN_DIST):
            continue
        added_poses.append(pose)
        model_name = '_'.join([surf['name'], str(surf_index), obj_name, str(obj_index)])
        success = spawn_model(
            name=model_name,
            model=models[obj_name],
            pose=pose,
            frame=surf['name'] + '_' + str(surf_index) + '::link'
        )
        if success:
            spawned[obj_name] += 1
        obj_index += 1


def spawn_surfaces(use_preferred_locs=False):
    added_poses = []  # to check that tables are at least SURFS_MIN_DIST apart from each other
    for surf in surfaces:
        surf_index = 0
        # clearance = surface diagonal / 2 + some extra margin
        clearance = sqrt((surf['size'][0] / 2.0) ** 2 + (surf['size'][1] / 2.0) ** 2) + 0.1
        while surf_index < surf['count'] and not rospy.is_shutdown():
            if use_preferred_locs:
                x, y = random.choice(PREFERRED_LOCATIONS)
            else:
                x = random.uniform(min_x, max_x)
                y = random.uniform(min_y, max_y)
            theta = random.uniform(-pi, +pi)
            pose = create_2d_pose(x, y, theta)
            # we check that the distance to all previously added surfaces is below a threshold to space the surfaces
            if close_to_prev_pose(pose, added_poses, SURFS_MIN_DIST):
                continue

            # check also that the surface is not too close to the robot
            if close_to_robot(pose, robot_pose, SURFS_MIN_DIST):
                continue

            # and in open space
            if close_to_obstacle(x, y, theta, clearance):
                continue

            added_poses.append(pose)
            model = surf['name']
            model_name = model + '_' + str(surf_index + 10)   # allow for some objects added by hand
            success = spawn_model(
                name=model_name,
                model=models[model],
                pose=pose,
                frame='ground_plane::link'
            )
            if success:
                # populate this surface with some objects
                spawn_objects(surf, surf_index + 10)
            surf_index += 1


def spawn_cats(use_preferred_locs=False):
    added_poses = []  # to check that cats are at least CATS_MIN_DIST apart from each other
    for cat in cats:
        cat_index = 0
        while cat_index < cat['count'] and not rospy.is_shutdown():
            if use_preferred_locs:
                x, y = random.choice(PREFERRED_LOCATIONS)
            else:
                x = random.uniform(min_x, max_x)
                y = random.uniform(min_y, max_y)
            theta = random.uniform(-pi, +pi)
            pose = create_2d_pose(x, y, theta)
            # we check that the distance to all previously added surfaces is below a threshold to space the surfaces
            if close_to_prev_pose(pose, added_poses, CATS_MIN_DIST):
                continue

            # check also that the surface is not too close to the robot
            if close_to_robot(pose, robot_pose, CATS_MIN_DIST):
                continue

            # and not within an obstacle
            if close_to_obstacle(x, y, theta, 0.0):
                continue

            added_poses.append(pose)
            model = cat['name']
            model_name = model + '_' + str(cat_index)
            success = spawn_model(
                name=model_name,
                model=models[model].format(name=model_name),
                pose=pose,
                frame='ground_plane::link'
            )
            if success:
                rospy.loginfo("Spawn %s at %s", model_name, pose2d2str(pose))
            cat_index += 1


def spawn_rockets():
    # spawn 10 x 10 rockets
    rocket_index = 1
    for i, j in product(range(-10, 0), range(10)):
        spawn_model(
            name='rocket' + str(rocket_index),
            model=models['rocket'],
            pose=create_2d_pose(i, j, 0),
            frame='ground_plane::link'
        )
        rocket_index += 1


def delete_all():
    for surf in surfaces:
        for surf_index in range(surf['count']):
            for obj in surf['objs']:
                for obj_index in range(obj['count']):
                    try:
                        model_name = '_'.join([surf['name'], str(surf_index + 10), obj['name'], str(obj_index)])
                        delete_model_client(model_name)
                    except rospy.ServiceException:
                        pass
                    obj_index += 1
            try:
                delete_model_client(surf['name'] + '_' + str(surf_index + 10))
            except:
                pass
            surf_index += 1


if __name__ == "__main__":
    rospy.init_node("spawn_gazebo_models")

    if len(sys.argv) == 1:
        rospy.logerr("Usage spawn_gazebo_models.py objects | cats | playground_objs | playground_cubes [-d] [-l]")
        sys.exit(-1)

    ros_pack = rospkg.RosPack()

    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client.wait_for_service(30)

    if len(sys.argv) > 2 and '-d' in sys.argv:
        # optionally delete previously spawned objects  TODO:  broken,, could call instead whenever a model fails to spawn
        delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_client.wait_for_service(30)
        delete_all()

    # get map bounds
    map_metadata = rospy.wait_for_message('map_metadata', MapMetaData, 30)
    min_x = map_metadata.origin.position.x
    min_y = map_metadata.origin.position.y
    max_x = map_metadata.origin.position.x + map_metadata.width * map_metadata.resolution
    max_y = map_metadata.origin.position.y + map_metadata.height * map_metadata.resolution

    load_models()

    robot_pose = TF2().transform_pose(None, 'base_footprint', 'map')

    random.seed()
    use_preferred_locs = len(sys.argv) > 2 and '-l' in sys.argv
    rospy.loginfo("Spawning %s in %s locations", sys.argv[1], 'preferred' if use_preferred_locs else 'random')
    if sys.argv[1] == 'objects':
        spawn_surfaces(use_preferred_locs)
        rospy.loginfo("Spawned objects:\n  " + '\n  '.join('{}: {}'.format(k, v) for k, v in spawned.items()))
    elif sys.argv[1] == 'cats':
        spawn_cats(use_preferred_locs)
        spawn_rockets()
    elif sys.argv[1] == 'playground_random':  # random objects over a random table
        surface = random.choice(surfaces)
        spawn_model(surface['name'] + '_0', models[surface['name']], create_2d_pose(0.44, 0, pi / 2.0),
                    'ground_plane::link')
        spawn_objects(surface, 0, sys.argv[2] if len(sys.argv) > 2 and not use_preferred_locs else None)
    elif sys.argv[1] == 'playground_objs':  # a sample of objects mostly at reachable locations
        spawn_model('lack_table', models['lack_table'], create_2d_pose(0.45, 0, 0.0), 'ground_plane::link')
        for obj in PLAYGROUND_OBJS:
            spawn_model(obj[0], models[obj[1]], create_3d_pose(*obj[2]), 'ground_plane::link')
    elif sys.argv[1] == 'playground_cubes':  # cubes at reachable locations, ready to stack
        spawn_model('doll_table', models['doll_table'], create_2d_pose(0.38, 0, 0.0), 'ground_plane::link')
        for obj in PLAYGROUND_CUBES:
            spawn_model(obj[0], models[obj[1]], create_3d_pose(*obj[2]), 'doll_table::link')

    sys.exit(0)
