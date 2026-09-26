/*
 * Author: Jorge Santos
 */

#include <rclcpp/rclcpp.hpp>

#include "thorp_manipulation/pick_and_place_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // The MoveIt configuration comes as parameter overrides, read by MoveIt's own loaders
  auto node = std::make_shared<rclcpp::Node>(
      "manipulation", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  thorp::manipulation::PickAndPlaceServer pick_and_place(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
