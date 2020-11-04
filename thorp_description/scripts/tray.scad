side = 150;
height = 20;
thickness = 2;
opening = 1.05;
side_w_walls = side + 2*thickness;

difference()
{
  union()
  {
    difference()
    {
      linear_extrude(height = height, scale = opening)
        translate([-side_w_walls/2, -side_w_walls/2, 0])
          square([side_w_walls, side_w_walls]);

      linear_extrude(height = height, scale = opening)
        translate([-side/2, -side/2, 0])
          square([side, side]);
    }

    translate([-side_w_walls/2, -side_w_walls/2, 0])
      cube([side_w_walls, side_w_walls, thickness]);
  }

  // holes for attaching screws
  translate([0, +33.7, -1])
    cylinder(r=2.3, h=20, center=true, $fn=20);
  translate([0, -33.7, -1])
    cylinder(r=2.3, h=20, center=true, $fn=20);
}