// 50% longer IKEA Lack table. Dimensions: length: 82.5 cm, width: 55 cm, height: 45 cm.

side_x = 825;
side_y = 550;
height = 450;
thickness = 50;

union()
{
  translate([-side_x/2, -side_y/2, height - thickness/2])
    cube([side_x, side_y, thickness]);

  translate([-side_x/2, -side_y/2, 0])
    cube([thickness, thickness, height]);

  translate([+side_x/2 - thickness, -side_y/2, 0])
    cube([thickness, thickness, height]);

  translate([+side_x/2 - thickness, +side_y/2 - thickness, 0])
    cube([thickness, thickness, height]);
  
  translate([-side_x/2, +side_y/2 - thickness, 0])
    cube([thickness, thickness, height]);
}
