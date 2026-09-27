#include "thorp_bt_cpp/bt_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bt_server");

  thorp::bt::Server server;
  if (!server.loadTrees())
    return EXIT_FAILURE;
  server.run();
  return EXIT_SUCCESS;
}
