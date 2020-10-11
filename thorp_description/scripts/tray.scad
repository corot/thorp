side = 150;
height = 15;
thickness = 2;

difference()
{
  translate([-side/2, -side/2, 0])
    cube([side, side, height]);
  translate([-side/2 + thickness, -side/2 + thickness, thickness])
    cube([side - 2*thickness, side - 2*thickness, height]);
}
