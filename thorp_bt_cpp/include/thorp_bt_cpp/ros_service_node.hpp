#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>
#include <ros/service_client.h>

namespace BT
{
/**
 * Base node to implement a ROS Service client node.
 * Templated parent class should be SyncActionNode for actions or ConditionNode for conditions
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

  /// These ports are mandatory; child classes must include them if they override this method
  static PortsList providedPorts()
  {
    return { InputPort<std::string>("service_name", "name of the ROS service"),
             InputPort<unsigned int>("timeout", 100, "timeout to connect to server (milliseconds)") };
  }

  /// User must implement this method.
  virtual void sendRequest(RequestType& request) = 0;

  /// Method (to be implemented by the user) to receive the reply.
  /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual NodeStatus onResponse(const ResponseType& rep) = 0;

  enum FailureCause
  {
    MISSING_SERVER = 0,
    FAILED_CALL = 1
  };

  /// Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus onFailedRequest(FailureCause failure)
  {
    return NodeStatus::FAILURE;
  }

protected:
  ros::ServiceClient service_client_;

  typename ServiceT::Response reply_;

  NodeStatus tick() override
  {
    if (!service_client_.isValid())
    {
      std::string server = ParentT::template getInput<std::string>("service_name").value();
      service_client_ = ros::NodeHandle().serviceClient<ServiceT>(server);
    }

    unsigned int msec = ParentT::template getInput<unsigned int>("timeout").value();
    ros::Duration timeout(static_cast<double>(msec) * 1e-3);

    bool connected = service_client_.waitForExistence(timeout);
    if (!connected)
    {
      return onFailedRequest(MISSING_SERVER);
    }

    typename ServiceT::Request request;
    sendRequest(request);
    bool received = service_client_.call(request, reply_);
    if (!received)
    {
      return onFailedRequest(FAILED_CALL);
    }
    return onResponse(reply_);
  }
};

}  // namespace BT
