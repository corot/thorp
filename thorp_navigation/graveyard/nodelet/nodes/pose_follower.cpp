/*
 * Node wrapping of pose_follower nodelet, in case you want to use it standalone.
 * Simpler than create a nodelet manager.
 * Author: Jorge Santos
 */

#include <ros/ros.h>
#include "thorp_navigation/pose_follower.h"  // TODO (if ever needed)


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_follower");

  thorp_navigation::PoseFollower pf;
  pf.onInit();

  ros::spin();
}
