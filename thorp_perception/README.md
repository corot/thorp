# Thorp perception

`object_detection` detects tables and tabletop objects on the Xtion point cloud:

- `object_detection` action (`thorp_msgs/DetectObjects`): segments the table in front of the robot and the objects
  on it, identifies each object by matching it against the templates in `meshes`, and adds all of them to the
  planning scene as collision objects.
- `object_detection/detect_tables` action (`thorp_msgs/DetectTables`): segments the table in front of the robot,
  if its shortest side reaches the given minimum.

`xtion_fov_analyzer.py` tells whether something close blocks the Xtion field of view.

## Code origin

The segmentation and template matching are rewritten from Thorp's forks of two GT-RAIL packages, both BSD licensed:

- `src/segmentation.cpp`: [corot/rail_segmentation](https://github.com/corot/rail_segmentation), `thorp` branch,
  commit c970d5ea572487bf52b3f9dde014b0a78a6a8961; forked from [GT-RAIL/rail_segmentation](https://github.com/GT-RAIL/rail_segmentation).
  Copyright (c) 2015, Worcester Polytechnic Institute.
- `src/template_matcher.cpp`: [corot/rail_mesh_icp](https://github.com/corot/rail_mesh_icp), `thorp` branch,
  commit 9e668934d3171a9058c79dee468001853eed8ad5; forked from [GT-RAIL/rail_mesh_icp](https://github.com/GT-RAIL/rail_mesh_icp).
  Copyright (c) 2019, Robot Autonomy and Interactive Learning.

The templates (`meshes/*.pcd`) were sampled from the object meshes with `rail_mesh_icp`'s mesh sampler.
