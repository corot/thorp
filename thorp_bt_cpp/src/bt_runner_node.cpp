#include "thorp_bt_cpp/bt_runner.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Trees read app parameters by name, without declaring them
  auto node = std::make_shared<rclcpp::Node>(
      "bt_runner", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
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
