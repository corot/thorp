#!/usr/bin/env python
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
             'size': (0.35, 0.35),
             'objs': 4,
             'dist': 'uniform',  # different distributions: 'uniform', 'diagonal', 'xor', '+/+'
             'count': 2},
            {'name': 'lack_table',
             'size': (0.45, 0.45),
             'objs': 6,
             'dist': 'uniform',  # different distributions: 'uniform', 'diagonal', 'xor', '+/+'
             'count': 2}
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

PLAYGROUND_OBJS = [('star', 'star', (0.28, 0.017, 0.5, 0.0, 0.0, 0.2)),
                   ('clover', 'clover', (0.36, -0.037, 0.5, 0.0, 0.0, 1.4)),
                   ('square', 'square', (0.29, -0.16, 0.5, 0.0, 0.0, 1.1)),
                   ('triangle', 'triangle', (0.32, 0.1, 0.5, 0.0, 0.0, 0.1)),
                   ('rectangle', 'rectangle', (0.27, 0.15, 0.5, 0.0, 0.0, 0.4)),
                   ('cross', 'cross', (0.37, 0.15, 0.5, 0.0, 0.0, 0.75)),
                   ('tower', 'tower', (0.38, -0.14, 0.5, 0.0, 0.0, 0.15)),
                   ('circle', 'circle', (0.48, 0.12, 0.5, 0.0, 0.0, 1.15)),
                   ('cube', 'cube', (0.46, 0.1, 0.5, 0.0, 0.0, 0.85))]

PLAYGROUND_CUBES = [('cube 1', 'cube', (-0.15, 0.017, 0.45, 0.0, 0.0, 0.2)),
                    ('cube 2', 'cube', (-0.11, -0.10, 0.45, 0.0, 0.0, 0.15)),
                    ('cube 3', 'cube', (-0.14, -0.16, 0.45, 0.0, 0.0, 1.1)),
                    ('cube 4', 'cube', (-0.12, 0.15, 0.45, 0.0, 0.0, 0.4)),
                    ('cube 5', 'cube', (-0.11, 0.10, 0.45, 0.0, 0.0, 0.85)),
                    ('circle', 'circle', (0.03, 0.12, 0.45, 0.0, 0.0, 1.15)),
                    ('clover', 'clover', (-0.02, -0.037, 0.45, 0.0, 0.0, 1.4)),
                    ('cross', 'cross', (0.08, -0.15, 0.45, 0.0, 0.0, 0.75)),
                    ('triangle', 'triangle', (0.13, 0.1, 0.45, 0.0, 0.0, 0.1))]

SURFS_MIN_DIST = 1.5
OBJS_MIN_DIST = 0.08
CATS_MIN_DIST = 5.0

PREFERRED_LOCATIONS = [(12.5, 7.5),
                       (8.4, 9.6),
                       (4.6, 9.8),
                       (5.8, 6.5)]


def spawn_model(name, model, pose, frame):
    """
    equivalent to command line:
    rosrun gazebo_ros spawn_model -sdf -database wood_cube_2_5cm -model wood_cube_2_5cm_10 -reference_frame doll_table_0::link -x 0 -y 0 -z 0.45
    """
    resp = spawn_model_client(
        model_name=name,
        model_xml=models[model],
        initial_pose=pose,
        reference_frame=frame
    )
    if not resp.success:
        rospy.logerr("Spawn model failed: %s", resp.status_message)
    return resp.success


def load_models():
    global models
    for obj in objects + [s['name'] for s in surfaces] + [c['name'] for c in cats] + ['rocket']:
        model_path = ros_pack.get_path('thorp_simulation') + '/worlds/gazebo/models/' + obj + '/model.sdf'
        models[obj] = open(model_path, 'r').read()


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


def close_to_obstacle(x, y, clearance):
    # check if the location is away from any non-zero cost in the global costmap by at least the given clearance
    # (we need to subtract robot radius because check pose service assumes we want to check the footprint cost)
    resp = check_pose_srv(pose=create_2d_pose(x, y, 0.0, 'map'), safety_dist=clearance - robot_radius,
                          costmap=CheckPoseRequest.GLOBAL_COSTMAP)
    if resp.state > 0 or resp.cost > 0:
        return True
    return False


def spawn_objects(surf, surf_index):
    added_poses = [create_3d_pose(0, 0, 0, 0, 0, 0)]  # fake pose to avoid the (non-reachable) surface's center
    obj_index = 0
    while obj_index < surf['objs'] and not rospy.is_shutdown():
        obj_name = random.choice(objects)

        # even distribution
        x = random.uniform(-surf['size'][0] / 2.0, +surf['size'][0] / 2.0)
        y = random.uniform(-surf['size'][1] / 2.0, +surf['size'][1] / 2.0)

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
            print("ERROR: unknown distribution " + surf['dist'])
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
            model=obj_name,
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
        clearance = sqrt((surf['size'][0] / 2.0) ** 2 + (surf['size'][1] / 2.0) ** 2) + 0.2
        while surf_index < surf['count'] and not rospy.is_shutdown():
            if use_preferred_locs:
                x, y = random.choice(PREFERRED_LOCATIONS)
            else:
                x = random.uniform(min_x, max_x)
                y = random.uniform(min_y, max_y)
            pose = create_2d_pose(x, y, random.uniform(-pi, +pi))
            # we check that the distance to all previously added surfaces is below a threshold to space the surfaces
            if close_to_prev_pose(pose, added_poses, SURFS_MIN_DIST):
                continue

            # check also that the surface is not too close to the robot
            if close_to_robot(pose, robot_pose, SURFS_MIN_DIST):
                continue

            # and in open space
            if close_to_obstacle(x, y, clearance):
                continue

            added_poses.append(pose)
            model_name = surf['name'] + '_' + str(surf_index + 10)   # allow for some objects added by hand
            success = spawn_model(
                name=model_name,
                model=surf['name'],
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
            pose = create_2d_pose(x, y, random.uniform(-pi, +pi))
            # we check that the distance to all previously added surfaces is below a threshold to space the surfaces
            if close_to_prev_pose(pose, added_poses, CATS_MIN_DIST):
                continue

            # check also that the surface is not too close to the robot
            if close_to_robot(pose, robot_pose, CATS_MIN_DIST):
                continue

            # and not within an obstacle
            if close_to_obstacle(x, y, 0.0):
                continue

            added_poses.append(pose)
            model_name = cat['name'] + '_' + str(cat_index)
            success = spawn_model(
                name=model_name,
                model=cat['name'],
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
            model='rocket',
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
        print("Usage spawn_gazebo_models.py objects | cats | playground_objs | playground_cubes [-d] [-l]")
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

    # we will use MBF's check pose service to ensure that the spawned surfaces are in open spaces
    check_pose_srv = rospy.ServiceProxy('move_base_flex/check_pose_cost', CheckPose, persistent=True)
    check_pose_srv.wait_for_service(10)

    load_models()

    robot_pose = TF2().transform_pose(None, 'base_footprint', 'map')
    robot_radius = rospy.get_param('move_base_flex/local_costmap/robot_radius')

    random.seed()
    use_preferred_locs = len(sys.argv) > 2 and '-l' in sys.argv
    if sys.argv[1] == 'objects':
        spawn_surfaces(use_preferred_locs)
        print("Spawned objects:\n  " + '\n  '.join('{}: {}'.format(k, v) for k, v in spawned.items()))
    elif sys.argv[1] == 'cats':
        spawn_cats(use_preferred_locs)
        spawn_rockets()
    elif sys.argv[1] == 'playground_objs':
        spawn_model('lack_table', 'lack_table', create_2d_pose(0.45, 0, 0.0), 'ground_plane::link')
        for obj in PLAYGROUND_OBJS:
            spawn_model(obj[0], obj[1], create_3d_pose(*obj[2]), 'ground_plane::link')
    elif sys.argv[1] == 'playground_cubes':
        spawn_model('doll_table', 'doll_table', create_2d_pose(0.38, 0, 0.0), 'ground_plane::link')
        for obj in PLAYGROUND_CUBES:
            spawn_model(obj[0], obj[1], create_3d_pose(*obj[2]), 'doll_table::link')

    sys.exit(0)
