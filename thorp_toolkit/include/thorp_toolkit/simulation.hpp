#pragma once

#include <rclcpp/rclcpp.hpp>

namespace thorp::toolkit
{

/**
 * Wait until the objects spawner node, that exits after spawning the simulated objects, is gone.
 * @param node Node to query the ROS graph with
 * @param timeout Maximum time to wait; zero to wait indefinitely
 */
void waitForObjectsSpawning(const rclcpp::Node::SharedPtr& node,
                            const rclcpp::Duration& timeout = rclcpp::Duration(0, 0));

};  // namespace thorp::toolkit
