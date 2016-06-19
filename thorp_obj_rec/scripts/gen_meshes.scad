// Basic meshes to populate or ORK database
// I must go object by object uncommenting, rendering (F6) and exporting to STL
// Things would improve if this PR gets merged: https://github.com/openscad/openscad/pull/1534

//cube_2_5();
//tower_5();
//lipstick();


module cube_2_5()
{
  translate([0, 0, 0.0125])
    cube(0.025, true);
}

module tower_5()
{
  translate([0, 0, 0.025])
    cube([0.025, 0.025, 0.05], true);
}

module lipstick()
{
  translate([0, 0, 0.037]) 
    cylinder(0.074, 0.01, 0.01, true, $fn=16);
}

