#include "thorp_toolkit/simulation.hpp"

namespace thorp::toolkit
{

void waitForObjectsSpawning(ros::Duration timeout)
{
  ros::Rate rate(5.0);
  ros::Time time_start = ros::Time::now();
  while (ros::ok() && (timeout.isZero() || (ros::Time::now() - time_start) < timeout))
  {
    std::vector<std::string> nodes;
    if (!ros::master::getNodes(nodes))
    {
      ROS_ERROR_STREAM("Get list of nodes failed");
      return;
    }
    if (std::find(nodes.begin(), nodes.end(), "/objects_spawner") == nodes.end())
    {
      return;
    }
    rate.sleep();
  }
  ROS_ERROR_STREAM("Objects spawning not completed after " << timeout << " seconds");
}

};  // namespace thorp::toolkit