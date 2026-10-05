/*
 * Author: Jorge Santos
 */

#include <rclcpp/rclcpp.hpp>

#include "thorp_manipulation/pick_and_place_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // The MoveIt configuration comes as parameter overrides, read by MoveIt's own loaders
  const auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("manipulation", options);
  // MoveIt's planning pipeline logs each planning step at INFO on its node's logger, so it plans on a node of its own
  auto planning_node = std::make_shared<rclcpp::Node>("manipulation_planning", options);
  thorp::manipulation::PickAndPlaceServer pick_and_place(node, planning_node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(planning_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
