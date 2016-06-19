// Basic meshes to populate or ORK database
// Set OBJ to the object you want to generate, or call from command line for batch processing:
// openscad -DOBJ='"cube_2_5"' -o cube_2_5.stl gen_meshes.scad
// openscad -DOBJ='"tower_5"'  -o tower_5.stl  gen_meshes.scad
// openscad -DOBJ='"lipstick"' -o lipstick.stl gen_meshes.scad

OBJ = undef;

if (OBJ == "cube_2_5") {
  translate([0, 0, 0.0125])
    cube(0.025, true);
}
if (OBJ == "tower_5") {
  translate([0, 0, 0.025])
    cube([0.025, 0.025, 0.05], true);
}
if (OBJ == "lipstick") {
  translate([0, 0, 0.037]) 
    cylinder(0.074, 0.01, 0.01, true, $fn=16);
}
