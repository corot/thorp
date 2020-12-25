#!/usr/bin/env python

# Generate meshes, pointclouds and gazebo models for all de objects in the list
# WARN: run the script from its containing folder, so it can find gen_meshes.scad

import rospkg
import subprocess
import os.path


def template(template_file, variables):
    return open(template_file, 'r').read().format(**variables)


try:
    # use rospkg to get the absolute path of meshes on thorp_perception
    rospack = rospkg.RosPack()

    # For each object, check if we already have a STL mesh. If not, generate it with OpenSCAD. We must convert to
    # binary, as OpenSCAD can only export ASCII STL by now (see https://github.com/openscad/openscad/issues/1555)
    # For already binary meshes, conversion just does nothing.
    # Last field is for gazebo material (see http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials)

    objects = [['coke', 'A universal can of coke', 'coke.stl', 'Red'],
               ['lipstick', 'Uriage lipstick, 7.4cm long', 'lipstick.stl', 'Blue'],
               ['tower', 'Untextured 2 x 2.5 cm side stacked cubes', 'tower.stl', 'SkyBlue'],
               ['cube', 'Untextured 2.5 cm side cube', 'cube.stl', 'White'],
               ['square', 'Untextured 3.2 cm side square', 'square.stl', 'Green'],
               ['rectangle', 'Untextured 2.8 x 3.8 cm rectangle', 'rectangle.stl', 'Yellow'],
               ['triangle', 'Untextured 3.3 cm side triangle', 'triangle.stl', 'Purple'],
               ['pentagon', 'Untextured 2.1 cm side pentagon', 'pentagon.stl', 'Indigo'],
               ['circle', 'Untextured 1.7 cm radius circle', 'circle.stl', 'Blue'],
               ['star', 'Untextured 1.7 cm radius, 5 points star', 'star.stl', 'Red'],
               ['diamond', 'Untextured 2.6 cm side diamond', 'diamond.stl', 'Orange'],
               ['cross', 'Untextured 3.7 x 3.7 cm cross', 'cross.stl', "Grey"],
               ['clover', 'Untextured 3.7 x 3.7 cm clover', 'clover.stl', 'Green']]

    meshes_path = rospack.get_path('thorp_perception') + "/meshes/"
    models_path = rospack.get_path('thorp_simulation') + "/worlds/gazebo/models/"
    for object in objects:
        # Run gen_meshes.scad to generate an STL mesh for each of the objects in the list
        if not os.path.exists(meshes_path + object[0] + '.stl'):
            ###   TODO  obj_name = object[2][:-4]  just need the list..  takes as param, w/ all as default
            print(subprocess.check_output(['openscad', '-DOBJ="' + object[0] + '"', '-o', 'tmp.ascii.stl',
                                           'generate_meshes.scad']))
            print(subprocess.check_output(['admesh', '-b', meshes_path + object[0] + '.stl', 'tmp.ascii.stl']))
            print(subprocess.check_output(['rm', 'tmp.ascii.stl']))

        # Run meshlabserver on each mesh to convert to ply format
        if not os.path.exists(meshes_path + object[0] + '.ply'):
            print(subprocess.check_output(['meshlabserver', '-i', meshes_path + object[0] + '.stl',
                                                            '-o', meshes_path + object[0] + '.ply']))

        # Run mesh sampler on each mesh to generate a pointcloud on pcd format
        if not os.path.exists(meshes_path + object[0] + '.pcd'):
            print(subprocess.check_output(['rosrun', 'rail_mesh_icp', 'mesh_sampler_node',
                                           meshes_path + object[0] + '.ply', meshes_path + object[0] + '.pcd']))

        # Create a gazebo model using template descriptors and linking the created meshes
        if not os.path.exists(models_path + object[0]):
            os.mkdir(models_path + object[0], 0o755)
            os.symlink(meshes_path + object[0] + '.stl', models_path + object[0] + '/model.stl')
            with open(models_path + object[0] + '/model.config', "w") as f:
                f.write(template(models_path + 'templates/model.config', {'name': object[0],
                                                                          'desc': object[1]}))
            with open(models_path + object[0] + '/model.sdf', "w") as f:
                f.write(template(models_path + 'templates/model.sdf', {'name': object[0],
                                                                       'color': object[3]}))

except rospkg.common.ResourceNotFound as err:
    print("get package path failed: " + str(err))
except subprocess.CalledProcessError as err:
    print("execute command failed: " + str(err))
