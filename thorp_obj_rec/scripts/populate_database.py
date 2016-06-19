#!/usr/bin/env python

# Populate Thorp objects database with a bunch of simple objects
# WARN: this script assumes that the database doesn't contain any of this objects; if so, the script will hung while prompting for overwrite 

import re
import rospkg
import subprocess

try:
    # use rospkg to get the absolute path of meshes on thorp_obj_rec
    rospack = rospkg.RosPack()

    # For each object, call object_add, get the generated object id and call mesh_add
    # The expected output of object_add is:   Stored new object with id: 9628d89172994443d3f7a535ca000abd  WARN: including a \n!

    objects = [['coke_can', 'A universal can of coke', 'coke.stl'],
               ['lipstick', 'Uriage lipstick, 7.4cm long', 'lipstick.stl'],
               ['tower 5cm', 'Untextured 2 x 2.5 cm side stacked cubes', 'tower_5.stl'],
               ['cube 2.5cm', 'Untextured 2.5 cm side cube', 'cube_2_5.stl']]

    for object in objects:
        print subprocess.check_output(['admesh', '-b', 'tmp.stl', rospack.get_path('thorp_obj_rec') + "/meshes/" + object[2]])
        out = subprocess.check_output(['rosrun', 'object_recognition_core', 'object_add.py', '-n', object[0], '-d', object[1], '--commit'])
        id = re.split(' ', out)[-1][:-1]
        print subprocess.check_output(['rosrun', 'object_recognition_core', 'mesh_add.py', id, 'tmp.stl', '--commit'])
        print subprocess.check_output(['rm', 'tmp.stl'])
except rospkg.common.ResourceNotFound as err:
    print "get package path failed: " + str(err)
except subprocess.CalledProcessError as err:
    print "execute command failed: " + str(err)
