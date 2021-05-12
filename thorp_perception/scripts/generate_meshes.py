#!/usr/bin/env python

# Generate meshes, pointclouds and gazebo models for all de objects in the list
# WARN: run the script from its containing folder, so it can find gen_meshes.scad
import sys
import shutil

import rospkg
import subprocess
import os.path


def template(template_file, variables):
    return open(template_file, 'r').read().format(**variables)


overwrite_models = len(sys.argv) > 1 and '-om' in sys.argv[1]

try:
    # use rospkg to get the absolute path of meshes on thorp_perception
    rospack = rospkg.RosPack()

    # For each object, check if we already have a STL mesh. If not, generate it with OpenSCAD. We must convert to
    # binary, as OpenSCAD can only export ASCII STL by now (see https://github.com/openscad/openscad/issues/1555)
    # For already binary meshes, conversion just does nothing.
    # Forth field is for gazebo material (see http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials)
    # The remaining values are the bounding box dimensions and mass, used to calculate the inertia tensor.

    objects = [['lipstick', 'Uriage lipstick, 7.4cm long', 'lipstick.stl', 'Blue', 0.01, 0.01, 0.074, 0.006],
               ['tower', 'Untextured 2 x 2.5 cm side stacked cubes', 'tower.stl', 'SkyBlue', 0.025, 0.025, 0.05, 0.0146],
               ['cube', 'Untextured 2.5 cm side cube', 'cube.stl', 'White', 0.025, 0.025, 0.025, 0.0073],
               ['square', 'Untextured 3.2 cm side square', 'square.stl', 'Green', 0.032, 0.011, 0.032, 0.006],
               ['rectangle', 'Untextured 2.8 x 3.8 cm rectangle', 'rectangle.stl', 'Yellow', 0.028, 0.011, 0.038, 0.006],
               ['triangle', 'Untextured 3.3 cm side triangle', 'triangle.stl', 'Purple', 0.033, 0.011, 0.028, 0.004],
               ['pentagon', 'Untextured 2.1 cm side pentagon', 'pentagon.stl', 'Indigo', 0.032, 0.011, 0.032, 0.006],
               ['circle', 'Untextured 1.7 cm radius circle', 'circle.stl', 'Blue', 0.032, 0.011, 0.032, 0.006],
               ['star', 'Untextured 1.7 cm radius, 5 points star', 'star.stl', 'Red', 0.032, 0.011, 0.032, 0.006],
               ['diamond', 'Untextured 2.6 cm side diamond', 'diamond.stl', 'Orange', 0.026, 0.011, 0.026, 0.006],
               ['cross', 'Untextured 3.7 x 3.7 cm cross', 'cross.stl', "Grey", 0.037, 0.011, 0.037, 0.005],
               ['clover', 'Untextured 3.7 x 3.7 cm clover', 'clover.stl', 'Green', 0.037, 0.011, 0.037, 0.006]]

    inertial_template = '<mass>{mass}</mass>\n' \
                        '          <pose>0 0 {hz} 0 0 0</pose>\n' \
                        '          <inertia>\n' \
                        '            <ixx>{ixx}</ixx>\n' \
                        '            <ixy>0</ixy>\n' \
                        '            <ixz>0</ixz>\n' \
                        '            <iyy>{iyy}</iyy>\n' \
                        '            <iyz>0</iyz>\n' \
                        '            <izz>{izz}</izz>\n' \
                        '          </inertia>'

    meshes_path = rospack.get_path('thorp_perception') + '/meshes/'
    models_path = rospack.get_path('thorp_simulation') + '/worlds/gazebo/models/'
    for object in objects:
        # Run gen_meshes.scad to generate an STL mesh for each of the objects in the list
        if not os.path.exists(meshes_path + object[0] + '.stl'):
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
                                           meshes_path + object[0] + '.ply', meshes_path + object[0] + '.pcd',
                                           '-n_samples', '10000', '-leaf_size', '0.0025']))

        # Create a gazebo model using template descriptors and linking the created meshes
        model_path = models_path + object[0]
        if overwrite_models:
            shutil.rmtree(model_path)
        if not os.path.exists(model_path):
            variables = {'mass': object[7],
                         'hz': object[6] / 2.0,  # half height
                         'ixx': object[7] / 12 * (object[5]**2 + object[6]**2),
                         'iyy': object[7] / 12 * (object[4]**2 + object[6]**2),
                         'izz': object[7] / 12 * (object[5]**2 + object[4]**2)}
            # bounding box inertia: https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors
            inertial = inertial_template.format(**variables)
            os.mkdir(model_path, 0o755)
            # create a symbolic link from model to mesh to avoid duplicated disk usage, but use a relative
            # path so it remains valid when cloning the repo somewhere else, e.g on building a docker image
            rel_mesh_path = os.path.relpath(meshes_path, model_path) + '/'
            os.symlink(rel_mesh_path + object[0] + '.stl', model_path + '/model.stl')
            with open(model_path + '/model.config', "w") as f:
                f.write(template(models_path + 'templates/model.config', {'name': object[0],
                                                                          'desc': object[1]}))
            with open(model_path + '/model.sdf', "w") as f:
                f.write(template(models_path + 'templates/model.sdf', {'name': object[0],
                                                                       'color': object[3],
                                                                       'inertial': inertial}))

except rospkg.common.ResourceNotFound as err:
    print("get package path failed: " + str(err))
except subprocess.CalledProcessError as err:
    print("execute command failed: " + str(err))
