#pragma once

#include <chrono>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace thorp::bt
{
/**
 * Call a service within a tick: the client has its own callback group and executor, so the call completes while the
 * tree's node isn't spinning.
 * @return The response; none if the service isn't available or doesn't answer, logging why
 */
template <typename ServiceT>
std::optional<typename ServiceT::Response> callService(const rclcpp::Node::SharedPtr& node, const std::string& name,
                                                       const typename ServiceT::Request& request)
{
  using namespace std::chrono_literals;
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_callback_group(callback_group, node->get_node_base_interface());
  auto client = node->create_client<ServiceT>(name, rclcpp::ServicesQoS(), callback_group);
  if (!client->wait_for_service(1s))
  {
    RCLCPP_ERROR(node->get_logger(), "Service %s not available", name.c_str());
    return std::nullopt;
  }
  auto future = client->async_send_request(std::make_shared<typename ServiceT::Request>(request));
  if (executor.spin_until_future_complete(future, 5s) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Service %s didn't answer", name.c_str());
    client->remove_pending_request(future);
    return std::nullopt;
  }
  return *future.get();
}

}  // namespace thorp::bt
