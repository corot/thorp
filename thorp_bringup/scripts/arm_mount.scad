
module arm_base()
{
difference()
{
  union()
  {
    translate([20.7, 0, 0])
      import("mount.stl", convexity=3);

    translate([77.7, 0, 0])
      import("mount.stl", convexity=3);

    translate([-6, -6.6, 0])
      cube([60, 61, 3.81]);
  }
  translate([0, 42.0, 0])
    cylinder(r=2.3, h=20, center=true, $fn=20);
}

}


module cannon_mount()
{
  union()
  {
    difference()
    {
      translate([-1, 30, -25.5])
        cube([5, 5, 25.5]);
      translate([-1, 32.5, -19.5]) rotate([0, 90, 0])
        cylinder(r=1, h=20, center=true, $fn=20);
    }
    difference()
    {
      translate([-1, 3, -25.5])
        cube([5, 5, 25.5]);
      translate([-1, 5.5, -19.5]) rotate([0, 90, 0])
        cylinder(r=1, h=20, center=true, $fn=20);
    }

    translate([-12, 8, -13])
      cube([20, 22, 13]);
  }
}

arm_base();
cannon_mount();

