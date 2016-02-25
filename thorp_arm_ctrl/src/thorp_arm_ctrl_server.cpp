/*
 * Author: Jorge Santos
 */

#include "thorp_arm_ctrl/move_to_target_server.hpp"
#include "thorp_arm_ctrl/pickup_object_server.hpp"
#include "thorp_arm_ctrl/place_object_server.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "thorp_arm_ctrl_server");

  // Create pickup, place and move_to_target action servers
  thorp_arm_ctrl::MoveToTargetServer mtt_server("move_to_target");
  thorp_arm_ctrl::PickupObjectServer pick_server("pickup_object");
  thorp_arm_ctrl::PlaceObjectServer  place_server("place_object");

  // Setup an asynchronous spinner as the move groups operations need continuous spinning
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
