#!/usr/bin/env python

import sys
import rospy
import rospkg

from math import pi, copysign
from random import uniform, randrange

from gazebo_msgs.srv import SpawnModel

from thorp_toolkit.geometry import distance_2d, create_3d_pose

# equivalent to command line:
# rosrun gazebo_ros spawn_model -sdf -database wood_cube_2_5cm -model wood_cube_2_5cm_10 -reference_frame doll_table_0::link -x 0 -y 0 -z 0.45

objects = [{'name': 'doll_table_0',
            'size': (0.35, 0.35),
            'objs': [{'name': 'wood_cube_2_5cm',
                      'count': 7}],
            'dist': 'uniform'}]  # different distributions: 'uniform', 'diagonal', 'xor', '+/+'
MIN_DIST = 0.08


def close_to_prev_pose(pose, added_poses):
    # check if the pose is closer than MIN_DIST to any of the previous poses
    # TODO I need something less naive to add more than 10 objects, e.g. spatial hash
    for prev_pose in added_poses:
        if distance_2d(pose, prev_pose) < MIN_DIST:
            return True
    return False


if __name__ == "__main__":
    rospy.init_node("spawn_objects")

    ros_pack = rospkg.RosPack()
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client.wait_for_service(30)

    for surf in objects:
        for obj in surf['objs']:
            model_path = ros_pack.get_path('thorp_simulation') + '/worlds/gazebo/models/' + obj['name'] + '/model.sdf'
            model_xml = open(model_path, 'r').read(),
            i = 0
            added_poses = [create_3d_pose(0, 0, 0, 0, 0, 0)]  # fake pose to avoid the (non-reachable) table's center
            while i < obj['count']:
                # even distribution
                x = uniform(-surf['size'][0]/2.0, +surf['size'][0]/2.0)
                y = uniform(-surf['size'][1]/2.0, +surf['size'][1]/2.0)

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
                # pose = create_3d_pose(uniform(-0.2, +0.2), uniform(-0.2, +0.2), 0.45, 0.0, 0.0, uniform(-pi, +pi))
                # avoid the (non-reachable) center of the table    wrong,,,, avoids a cross
                # x = uniform(0.075, 0.2) * [-1, +1][randrange(2)]
                # y = uniform(0.075, 0.2) * [-1, +1][randrange(2)]
                # z = 0.45
                #pose = create_3d_pose(x, y, z, 0, 0, uniform(-pi, +pi))
                # we check that the distance to all previously added objects is below a threshold to space the objects
                if close_to_prev_pose(pose, added_poses):
                    continue
                added_poses.append(pose)
                spawn_model_client(
                    model_name=obj['name'] + '_' + str(i + 10),  # allow for some objects added by hand
                    model_xml=model_xml[0],
                    initial_pose=pose,
                    reference_frame=surf['name']+'::link'
                )
                i += 1
