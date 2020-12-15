#!/usr/bin/env python

import sys
import rospy
import rospkg

from math import pi, copysign
from random import uniform

from nav_msgs.msg import MapMetaData
from mbf_msgs.srv import CheckPose, CheckPoseRequest
from gazebo_msgs.srv import SpawnModel

from thorp_toolkit.geometry import distance_2d, create_2d_pose, create_3d_pose

# equivalent to command line:
# rosrun gazebo_ros spawn_model -sdf -database wood_cube_2_5cm -model wood_cube_2_5cm_10 -reference_frame doll_table_0::link -x 0 -y 0 -z 0.45

objects = [{'name': 'doll_table',
            'size': (0.35, 0.35),
            'objs': [{'name': 'wood_cube_2_5cm',
                      'count': 7}],
            'dist': 'uniform',  # different distributions: 'uniform', 'diagonal', 'xor', '+/+'
            'count': 3}]
SURFS_MIN_DIST = 1.5
OBJS_MIN_DIST = 0.08


def close_to_prev_pose(pose, added_poses, min_dist):
    # check if the pose is closer than MIN_DIST to any of the previous poses
    # TODO I need something less naive to add more than 10 objects, e.g. spatial hash
    for prev_pose in added_poses:
        if distance_2d(pose, prev_pose) < min_dist:
            return True
    return False


def close_to_obstacle(cx, cy, surf):
    # check if the surface is far way from any obstacle in the global costmap
    # we check the robot footprint in 9 poses, being the central one the surface pose;
    # if the footprint is not entirely on fully free space (0 cost), we reject the pose
    x_offset = surf['size'][0] / 2.0 + 0.2
    y_offset = surf['size'][1] / 2.0 + 0.2
    for x in [cx - x_offset, cx, cx + x_offset]:
        for y in [cy - y_offset, cy, cy + y_offset]:
            resp = check_pose_srv(pose=create_2d_pose(x, y, 0.0, 'map'), costmap=CheckPoseRequest.GLOBAL_COSTMAP)
            if resp.state > 0 or resp.cost > 0:
                return True
    return False


def spawn_objects(surf, surf_index):
    for obj in surf['objs']:
        model_path = ros_pack.get_path('thorp_simulation') + '/worlds/gazebo/models/' + obj['name'] + '/model.sdf'
        model_xml = open(model_path, 'r').read(),
        i = 0
        added_poses = [create_3d_pose(0, 0, 0, 0, 0, 0)]  # fake pose to avoid the (non-reachable) table's center
        while i < obj['count'] and not rospy.is_shutdown():
            # even distribution
            x = uniform(-surf['size'][0] / 2.0, +surf['size'][0] / 2.0)
            y = uniform(-surf['size'][1] / 2.0, +surf['size'][1] / 2.0)

            if surf['dist'] == 'diagonal':
                # half table by diagonal
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

            z = 0.45
            pose = create_3d_pose(x, y, z, 0, 0, uniform(-pi, +pi))
            # we check that the distance to all previously added objects is below a threshold to space the objects
            if close_to_prev_pose(pose, added_poses, OBJS_MIN_DIST):
                continue
            added_poses.append(pose)
            resp = spawn_model_client(
                model_name=obj['name'] + '_' + str(surf_index) + '_' + str(i),
                model_xml=model_xml[0],
                initial_pose=pose,
                reference_frame=surf['name'] + '_' + str(surf_index) + '::link'
            )
            if not resp.success:
                rospy.logerr("Spawn object failed: %s", resp.status_message)
                break
            i += 1


def spawn_surfaces(surfaces):
    added_poses = []  # to check that they are at least SURFS_MIN_DIST apart from each other

    for surf in surfaces:
        model_path = ros_pack.get_path('thorp_simulation') + '/worlds/gazebo/models/' + surf['name'] + '/model.sdf'
        model_xml = open(model_path, 'r').read(),
        i = 0
        while i < surf['count'] and not rospy.is_shutdown():
            x = uniform(min_x, max_x)
            y = uniform(min_y, max_y)
            z = 0.0
            pose = create_3d_pose(x, y, z, 0, 0, 0)  # uniform(-pi, +pi))  TODO cannot detect surface orientation by now
            # we check that the distance to all previously added surfaces is below a threshold to space the surfaces
            if close_to_prev_pose(pose, added_poses, SURFS_MIN_DIST):
                continue

            # check also that the surface is in open space
            if close_to_obstacle(x, y, surf):
                continue

            added_poses.append(pose)
            model_name = surf['name'] + '_' + str(i + 10)  # allow for some objects added by hand
            resp = spawn_model_client(
                model_name=model_name,
                model_xml=model_xml[0],
                initial_pose=pose,
                reference_frame='ground_plane::link'
            )
            if not resp.success:
                rospy.logerr("Spawn surface failed: %s", resp.status_message)
                break

            # populate this surface with some objects
            spawn_objects(surf, i + 10)

            i += 1


if __name__ == "__main__":
    rospy.init_node("spawn_objects")

    ros_pack = rospkg.RosPack()
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client.wait_for_service(30)

    # get map bounds
    map_metadata = rospy.wait_for_message('map_metadata', MapMetaData, 10)
    min_x = map_metadata.origin.position.x
    min_y = map_metadata.origin.position.y
    max_x = map_metadata.origin.position.x + map_metadata.width * map_metadata.resolution
    max_y = map_metadata.origin.position.y + map_metadata.height * map_metadata.resolution

    # we will use MBF's check pose service to ensure that the spawned tables are in open spaces
    check_pose_srv = rospy.ServiceProxy('move_base_flex/check_pose_cost', CheckPose, persistent=True)
    check_pose_srv.wait_for_service(10)

    spawn_surfaces(objects)