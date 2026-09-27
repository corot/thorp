#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

namespace BT
{
/**
 * Base node to implement a ROS Service client node, calling the service synchronously within a tick.
 * Templated parent class should be SyncActionNode for actions or ConditionNode for conditions.
 * The client has its own callback group and executor, so the call completes while the tree's node is not spinning.
 */
template <class ServiceT, class ParentT>
class RosServiceNode : public ParentT
{
protected:
  RosServiceNode(const std::string& name, const NodeConfig& conf) : ParentT(name, conf)
  {
  }

public:
  using BaseClass = RosServiceNode<ServiceT, ParentT>;
  using ParentType = ParentT;
  using ServiceType = ServiceT;
  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  RosServiceNode() = delete;

  ~RosServiceNode() override = default;

  /**
   * @brief ROS service client node default ports
   * These ports are mandatory; child classes must include them if they override this method
   */
  static PortsList providedPorts()
  {
    return { InputPort<std::string>("service_name", "name of the ROS service"),
             InputPort<unsigned int>("timeout", 100, "timeout to connect to server (milliseconds)") };
  }

  /**
   * @brief Fill the request to send to the service.
   */
  virtual void sendRequest(RequestType& request) = 0;

  /**
   * @brief Process the response from the service.
   * @return Node status
   */
  virtual NodeStatus onResponse(const ResponseType& rep) = 0;

  enum FailureCause
  {
    MISSING_SERVER = 0,
    FAILED_CALL = 1
  };

  /**
   * @brief Called when a service call failed. Can be overridden by the user.
   * @return Node status
   */
  virtual NodeStatus onFailedRequest(FailureCause failure)
  {
    return NodeStatus::FAILURE;
  }

protected:
  /** Maximum time to wait for a response */
  static constexpr std::chrono::seconds CALL_TIMEOUT{ 10 };

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  NodeStatus tick() override
  {
    if (!service_client_)
    {
      rclcpp::Node::SharedPtr node = ParentT::config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
      callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
      callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());
      std::string server = ParentT::template getInput<std::string>("service_name").value();
      service_client_ = node->create_client<ServiceT>(server, rclcpp::ServicesQoS(), callback_group_);
    }

    unsigned int msec = ParentT::template getInput<unsigned int>("timeout").value();
    if (!service_client_->wait_for_service(std::chrono::milliseconds(msec)))
    {
      return onFailedRequest(MISSING_SERVER);
    }

    auto request = std::make_shared<RequestType>();
    sendRequest(*request);
    auto future = service_client_->async_send_request(request);
    if (callback_group_executor_.spin_until_future_complete(future, CALL_TIMEOUT) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      service_client_->remove_pending_request(future);
      return onFailedRequest(FAILED_CALL);
    }
    return onResponse(*future.get());
  }
};

}  // namespace BT
