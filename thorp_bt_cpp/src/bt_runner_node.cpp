#include "thorp_bt_cpp/bt_runner.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bt_runner");

  thorp::bt::Runner runner;
  runner.run();
  return EXIT_SUCCESS;
}
