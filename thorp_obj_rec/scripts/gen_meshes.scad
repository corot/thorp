// Basic meshes to populate or ORK database
// Set OBJ to the object you want to generate, or call from command line for batch processing:
// openscad -DOBJ='"cube"' -o cube.stl gen_meshes.scad
// openscad -DOBJ='"tower"' -o tower.stl  gen_meshes.scad
// openscad -DOBJ='"lipstick"' -o lipstick.stl gen_meshes.scad

OBJ = undef;

use <parametric_star.scad>

if (OBJ == "cube") {
  translate([0, 0, 0.0125])
    cube(0.025, true);
}
if (OBJ == "tower") {
  translate([0, 0, 0.025])
    cube([0.025, 0.025, 0.05], true);
}
if (OBJ == "lipstick") {
  translate([0, 0, 0.037])
    cylinder(0.074, 0.01, 0.01, true, $fn=16);
}
if (OBJ == "square") {
  translate([0, 0, 0.016])
    cube([0.032, 0.011, 0.032], true);
}
if (OBJ == "rectangle") {
  translate([0, 0, 0.019])
    cube([0.028, 0.011, 0.038], true);
}
if (OBJ == "triangle") {
  translate([0, 0.0055, 0.0085])
  rotate([0, -90, 90])
  linear_extrude(height=0.011)
    circle(0.017, $fn=3);
}
if (OBJ == "pentagon") {
  translate([0, 0.0055, 0.0138])
  rotate([0, -90, 90])
  linear_extrude(height=0.011)
    circle(0.017, $fn=5);
}
if (OBJ == "circle") {
  translate([0, 0.0055, 0.017])
  rotate([0, -90, 90])
  linear_extrude(height=0.011)
    circle(0.017, $fn=64);
}
if (OBJ == "star") {
  translate([0, 0.0055, 0.0138])
  rotate([0, -90, 90])
    parametric_star(ri=0.017/2.5, re=0.017, h=0.011);
}
if (OBJ == "diamond") {
  translate([0.0145, 0.0055, 0])
  rotate([0, -90, 90])
  linear_extrude(height=0.011)
    polygon([[0,0],[0,0.026],[0.026,0.029],[0.026,0.003]]);
}
if (OBJ == "cross") {
  translate([0.006, 0.006, 0.012])
  rotate([0, -90, 90])
  union() {
    translate([0.012, 0, 0])
        cube(0.012);
    translate([0, 0.012, 0])
        cube(0.012);
    translate([-0.012, 0, 0])
        cube(0.012);
    translate([0, -0.012, 0])
        cube(0.012);
        cube(0.012);
  }
}
if (OBJ == "clover") {
  translate([0, 0.0055, 0.016])
  rotate([0, -90, 90])
  union() {
    translate([0.0075, 0.0075, 0])
    linear_extrude(height=0.011)
        circle(0.0085, $fn=32);
    translate([0.0075, -0.0075, 0])
    linear_extrude(height=0.011)
        circle(0.0085, $fn=32);
    translate([-0.0075, 0.0075, 0])
    linear_extrude(height=0.011)
        circle(0.0085, $fn=32);
    translate([-0.0075, -0.0075, 0])
    linear_extrude(height=0.011)
        circle(0.0085, $fn=32);
    linear_extrude(height=0.011)
        circle(0.0085, $fn=32);
  }
}
