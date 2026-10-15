#include "thorp_bt_cpp/bt_server.hpp"

#include <thorp_toolkit/common.hpp>
#include <thorp_toolkit/tf2.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Trees read app parameters by name, without declaring them, so callers can set any
  auto node = std::make_shared<rclcpp::Node>(
      "bt_server",
      rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
  thorp::toolkit::init(node);
  // Listen to TF from the start, as a buffer only holds the transforms received since it was created, and trees
  // look up transforms at the stamps of data received before
  thorp::toolkit::TF2::instance();
  thorp::bt::Server server(node);
  if (!server.loadTrees())
  {
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  server.start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
