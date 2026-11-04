#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

namespace BT
{
/**
 * Base node to implement a ROS action client node. It waits for its server when the tree is built, so a missing
 * server fails the tree's creation saying so; then it sends the goal getGoal gives when ticked, and runs until the
 * action ends, as onSucceeded, onAborted or onCanceled say. Halting it cancels the goal.
 * The client has its own callback group and executor, so it works while the tree's node is not spinning.
 */
template <class ActionT>
class RosActionNode : public StatefulActionNode
{
public:
  using BaseClass = RosActionNode<ActionT>;
  using ActionType = ActionT;
  using Goal = typename ActionT::Goal;
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  /**
   * @param action_name Server's name, unless the action_name port gives another one
   */
  RosActionNode(const std::string& name, const std::string& action_name, const NodeConfig& conf)
    : StatefulActionNode(name, conf)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    action_name_ = getInput<std::string>("action_name").value_or("");
    if (action_name_.empty())
      action_name_ = action_name;
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name_, callback_group_);
    const auto timeout = config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");
    if (!action_client_->wait_for_action_server(timeout))
      throw RuntimeError("Action server ", action_name_, " not available");
  }

  RosActionNode() = delete;

  ~RosActionNode() override = default;

  /**
   * @brief Ports of every action client node, with those the child adds
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    addition.insert(
        InputPort<std::string>("action_name", "", "name of the action server; the node's default if empty"));
    return addition;
  }

  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

protected:
  /** Maximum time for the server to accept or refuse a goal, and to answer a cancel request */
  static constexpr std::chrono::seconds GOAL_TIMEOUT{ 5 };
  static constexpr std::chrono::seconds CANCEL_TIMEOUT{ 1 };

  /**
   * @brief The goal to send, read from the input ports.
   */
  virtual Goal getGoal() = 0;

  /**
   * @brief Called with each feedback message from the server.
   */
  virtual void onFeedback(const Feedback& /*feedback*/)
  {
  }

  /**
   * @brief Called when the action succeeds.
   * @return SUCCESS by default
   */
  virtual NodeStatus onSucceeded(const Result& /*result*/)
  {
    return NodeStatus::SUCCESS;
  }

  /**
   * @brief Called when the action is aborted.
   * @return FAILURE by default
   */
  virtual NodeStatus onAborted(const Result& /*result*/)
  {
    return NodeStatus::FAILURE;
  }

  /**
   * @brief Called when the action is canceled by someone else; halting the node cancels it without calling this.
   * @return SUCCESS by default
   */
  virtual NodeStatus onCanceled()
  {
    return NodeStatus::SUCCESS;
  }

  /**
   * @brief Called when the server refuses the goal, or doesn't answer within GOAL_TIMEOUT.
   * @return FAILURE by default
   */
  virtual NodeStatus onRejected()
  {
    return NodeStatus::FAILURE;
  }

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;

private:
  rclcpp::Logger clientLogger() const
  {
    return node_->get_logger().get_child(name());
  }

  NodeStatus onStart() final
  {
    typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
    options.feedback_callback = [this](const typename GoalHandle::SharedPtr& /*goal_handle*/,
                                       const std::shared_ptr<const Feedback>& feedback) { onFeedback(*feedback); };
    auto future = action_client_->async_send_goal(getGoal(), options);
    if (callback_group_executor_.spin_until_future_complete(future, GOAL_TIMEOUT) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(clientLogger(), "Action server %s didn't answer the goal", action_name_.c_str());
      return onRejected();
    }
    goal_handle_ = future.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(clientLogger(), "Action server %s refused the goal", action_name_.c_str());
      return onRejected();
    }
    result_future_ = action_client_->async_get_result(goal_handle_);
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() final
  {
    callback_group_executor_.spin_some();
    if (result_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
      return NodeStatus::RUNNING;
    goal_handle_.reset();
    const auto wrapped_result = result_future_.get();
    switch (wrapped_result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return onSucceeded(*wrapped_result.result);
      case rclcpp_action::ResultCode::ABORTED:
        return onAborted(*wrapped_result.result);
      case rclcpp_action::ResultCode::CANCELED:
        return onCanceled();
      default:
        RCLCPP_ERROR(clientLogger(), "Action server %s ended the goal with an unknown result", action_name_.c_str());
        return NodeStatus::FAILURE;
    }
  }

  void onHalted() final
  {
    if (!goal_handle_)
      return;
    auto future = action_client_->async_cancel_goal(goal_handle_);
    if (callback_group_executor_.spin_until_future_complete(future, CANCEL_TIMEOUT) !=
        rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_WARN(clientLogger(), "Action server %s didn't answer the cancel request", action_name_.c_str());
    goal_handle_.reset();
  }

  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  typename GoalHandle::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandle::WrappedResult> result_future_;
};

}  // namespace BT
