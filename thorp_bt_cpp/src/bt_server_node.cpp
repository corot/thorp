#include "thorp_bt_cpp/bt_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Trees read app parameters by name, without declaring them
  auto node = std::make_shared<rclcpp::Node>(
      "bt_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
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
