#include "thorp_toolkit/simulation.hpp"

#include <algorithm>
#include <thread>

namespace thorp::toolkit
{
void waitForObjectsSpawning(const rclcpp::Node::SharedPtr& node, const rclcpp::Duration& timeout)
{
  const rclcpp::Time time_start = node->now();
  while (rclcpp::ok() && (timeout.nanoseconds() == 0 || (node->now() - time_start) < timeout))
  {
    auto nodes = node->get_node_names();
    if (std::find(nodes.begin(), nodes.end(), "/objects_spawner") == nodes.end())
    {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  RCLCPP_ERROR_STREAM(node->get_logger(), "Objects spawning not completed after " << timeout.seconds() << " seconds");
}
};  // namespace thorp::toolkit
