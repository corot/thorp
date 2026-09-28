#include "thorp_bt_cpp/bt_runner.hpp"

#include <thorp_toolkit/common.hpp>

int main(int argc, char** argv)
{
  // The node is named after the app, given as the first argument; a launch file node name would rename also the
  // nodes some tree nodes create, e.g. MoveIt's planning scene interface
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  const std::string app_name = args.size() > 1 ? args[1] : "bt_runner";
  // Trees read app parameters by name, without declaring them
  auto node = std::make_shared<rclcpp::Node>(
      app_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  thorp::toolkit::init(node);
  thorp::bt::Runner runner(node);
  int result = EXIT_FAILURE;
  if (runner.loadTree())
  {
    runner.run();
    result = EXIT_SUCCESS;
  }
  rclcpp::shutdown();
  return result;
}
