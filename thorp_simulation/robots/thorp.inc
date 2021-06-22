define kinect ranger
(
  sensor
  (
    range [ 0.4 6.5 ]
    fov 58.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define thorp position
(
  bitmap "../../robots/thorp_footprint.png"

  size [ 0.33 0.33 0.09 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  odom_error [ 0.03 0.03 0.0 3.0 ]
  gui_nose 1
  drive "diff"
  color "black"

  kinect(pose [ -0.1 0.0 0.2 0.0 ])
)
