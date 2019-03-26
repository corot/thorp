#!/usr/bin/env python

# Populate Thorp objects database with a bunch of simple objects.
# Install Couch Db: https://wg-perception.github.io/object_recognition_core/infrastructure/couch.html
# WARN 1: this script assumes that the database doesn't contain any of this objects; if so, the script will hung
# while prompting for overwrite
# WARN 2: run the script from its containing folder, so it can find gen_meshes.scad

import re
import rospkg
import subprocess
import os.path

try:
    # use rospkg to get the absolute path of meshes on thorp_obj_rec
    rospack = rospkg.RosPack()

    # For each object, check if we already have a STL mesh. If not, generate it with OpenSCAD. We must convert to
    # binary, as OpenSCAD can only export ASCII STL by now (see https://github.com/openscad/openscad/issues/1555)
    # For already binary meshes, conversion just does nothing.
    # Once we have our mesh, we call object_recognition_core object_add, get the generated object id and call mesh_add
    # Expected output of object_add: Stored new object with id: 9628d89172994443d3f7a535ca000abd  WARN: including a \n!

    objects = [['coke_can', 'A universal can of coke', 'coke.stl'],
               ['lipstick', 'Uriage lipstick, 7.4cm long', 'lipstick.stl'],
               ['tower', 'Untextured 2 x 2.5 cm side stacked cubes', 'tower.stl'],
               ['cube', 'Untextured 2.5 cm side cube', 'cube.stl'],
               ['square', 'Untextured 3.2 cm side square', 'square.stl'],
               ['rectangle', 'Untextured 2.8 x 3.8 cm rectangle', 'rectangle.stl'],
               ['triangle', 'Untextured 3.3 cm side triangle', 'triangle.stl'],
               ['pentagon', 'Untextured 2.1 cm side pentagon', 'pentagon.stl'],
               ['circle', 'Untextured 1.7 cm radius circle', 'circle.stl'],
               ['star', 'Untextured 1.7 cm radius, 5 points star', 'star.stl'],
               ['diamond', 'Untextured 2.6 cm side diamond', 'diamond.stl'],
               ['cross', 'Untextured 3.7 x 3.7 cm cross', 'cross.stl'],
               ['clover', 'Untextured 3.7 x 3.7 cm clover', 'clover.stl']]

    for object in objects:
        if os.path.exists(rospack.get_path('thorp_obj_rec') + "/meshes/" + object[2]):
            print subprocess.check_output(['cp', rospack.get_path('thorp_obj_rec') + "/meshes/" + object[2], 'tmp.ascii.stl'])
        else:
            print subprocess.check_output(['openscad', '-DOBJ="' + object[2][:-4] + '"', '-o', 'tmp.ascii.stl', 'gen_meshes.scad'])
        print subprocess.check_output(['admesh', '-b', 'tmp.bin.stl', 'tmp.ascii.stl'])
        out = subprocess.check_output(['rosrun', 'object_recognition_core', 'object_add.py', '-n', object[0], '-d', object[1], '--commit'])
        id = re.split(' ', out)[-1][:-1]
        print out
        print subprocess.check_output(['rosrun', 'object_recognition_core', 'mesh_add.py', id, 'tmp.bin.stl', '--commit'])
        print subprocess.check_output(['rm', 'tmp.ascii.stl', 'tmp.bin.stl'])
except rospkg.common.ResourceNotFound as err:
    print "get package path failed: " + str(err)
except subprocess.CalledProcessError as err:
    print "execute command failed: " + str(err)
