/*
 * Author: Jorge Santos
 */

#include "thorp_manipulation/move_to_target_server.hpp"
#include "thorp_manipulation/pickup_object_server.hpp"
#include "thorp_manipulation/place_object_server.hpp"
#include "thorp_manipulation/house_keeping_server.hpp"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "manipulation");

  // Create pickup, place and move_to_target action servers
  thorp::manipulation::HouseKeepingServer hk_server;
  thorp::manipulation::MoveToTargetServer mtt_server("move_to_target");
  thorp::manipulation::PickupObjectServer pick_server("pickup_object");
  thorp::manipulation::PlaceObjectServer  place_server("place_object");

  // Setup an asynchronous spinner as the move groups operations need continuous spinning
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
