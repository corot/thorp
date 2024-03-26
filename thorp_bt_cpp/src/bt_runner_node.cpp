#include "thorp_bt_cpp/bt_runner.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bt_runner");

  thorp::bt::Runner runner;
  if (!runner.loadTree())
    return EXIT_FAILURE;
  runner.run();
  return EXIT_SUCCESS;
}
