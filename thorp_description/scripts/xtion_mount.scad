
module xtion_mount()
{
  union()
  {
    translate([0, 0, 0])
      rotate([135, 0, 0])
        import("../meshes/xtion_mount_a45.stl", convexity=3);
    
    
    translate([-98, 10.122, 7.876])
      rotate([135, 0, 0])
        cube([196, 2, 2]);

    difference()
    {    
      difference()
      {    
        hull()
        {
            // place 4 circles in the corners, with the given radius
          translate([-98, -7.92, -8.818])
            rotate([0, 90, 0])
              cylinder(r=1,h=196, $fn=20);
        
          translate([-98, 9.3, -8.818])
            rotate([0, 90, 0])
              cylinder(r=1,h=196, $fn=20);
        
          translate([-98, 9.3, 7.28])
            rotate([0, 90, 0])
              cylinder(r=1,h=196, $fn=20);
        }

        union()
        {
          translate([-98, 0, -15])
            rotate([0, 0, 0])
              cylinder(r=8,h=30, $fn=20);
    
          translate([98, 0, -15])
            rotate([0, 0, 0])
              cylinder(r=8,h=30, $fn=20);
        }
      }

      translate([-20, 20, -20])
        rotate([45, 0, 0])
          hull()
          {
            // place 4 cylinders in the corners
            translate([15.25, 7.32, 0])
              cylinder(r=2,h=96, $fn=20);

            translate([15.25, -4.075, 0])
              cylinder(r=2,h=96, $fn=20);

            translate([-10.54, 7.32, 0])
              cylinder(r=2,h=96, $fn=20);

            translate([-10.54, -4.075, 0])
              cylinder(r=2,h=96, $fn=20);
          }

    }
  }
}

/* Uncomment to tilt till 50 degrees
scale([1, 0.83, 1]) 
  scale([1, 1/0.83, 1/0.83]) */
xtion_mount();
