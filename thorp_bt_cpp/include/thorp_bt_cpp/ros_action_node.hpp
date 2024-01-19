#pragma once

// behaviortree_cpp
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/tree_node.h>

// ros
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/action_definition.h>

// boost
#include <boost/function.hpp>

namespace BT
{
/**
 * Base Action to implement a ROS Action client node
 */
template <class ActionT>
class RosActionNode : public ActionNodeBase
{
protected:
  using GoalType = typename ActionT::_action_goal_type::_goal_type;
  using ResultType = typename ActionT::_action_result_type::_result_type;
  using FeedbackType = typename ActionT::_action_feedback_type::_feedback_type;
  typedef boost::shared_ptr<const FeedbackType> FeedbackConstPtr;
  typedef boost::shared_ptr<const ResultType> ResultConstPtr;

  RosActionNode(const std::string& xml_tag_name, const NodeConfiguration& conf) : ActionNodeBase(xml_tag_name, conf)
  {
  }

private:
  static constexpr char LOGNAME[] = "RosActionNode";

  inline bool createActionClient(double timeout)
  {
    action_client_ = std::make_unique<actionlib::SimpleActionClient<ActionT>>(action_name_, true);
    ROS_INFO_STREAM(LOGNAME << ": Waiting for action server " << action_name_);

    // TODO refactor this to not block the tree (e.g. change status to running while waiting)
    // while support infinite waiting (e.g. when timeout < 0)
    if (action_client_->waitForServer(ros::Duration(timeout)))
    {
      ROS_INFO_STREAM(LOGNAME << ": Action server " << action_name_ << " found");
      return true;
    }
    else
    {
      ROS_ERROR_STREAM(LOGNAME << ": Action server " << action_name_ << " not found");
      this->onServerUnavailable();
      return false;
    }
  }

  inline bool isActive() const
  {
    // TODO need this fix too
    return action_client_ && !action_client_->getState().isDone();
  }

  inline void feedbackCb(const FeedbackConstPtr& feedback)
  {
    onFeedback(feedback);
  }

  /**
   * @brief Function to perform some user-defined operation if server is not available
   */
  virtual void onServerUnavailable()
  {
  }

  /**
   * @brief Function to perform some user-defined operation on tick.
   */
  virtual void onTick()
  {
  }

  /**
   * @brief Function to perform some user-defined operation on the latest feedback message from the action server.
   */
  virtual void onFeedback(const FeedbackConstPtr& feedback)
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual NodeStatus onSucceeded(const ResultConstPtr& res)
  {
    return NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation whe the action is aborted.
   * @return NodeStatus Returns FAILURE by default, user may override return another value
   */
  virtual NodeStatus onAborted(const ResultConstPtr& res)
  {
    return NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual NodeStatus onCanceled()
  {
    return NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation when the action finishes, independently of the outcome.
   * This gets called right before onSucceeded, onAborted and onCanceled, so it's the right place to do cleanups
   * required regardless of what happened.
   */
  virtual void onFinished()
  {
  }

  /**
   * @brief Function to perform some user-defined operation when the goal is rejected by server.
   * @return NodeStatus Returns FAILURE by default, user may override return another value
   */
  virtual NodeStatus onRejected()
  {
    return NodeStatus::FAILURE;
  }

public:
  using ActionType = ActionT;

  RosActionNode() = delete;
  ~RosActionNode() override = default;

  /**
   * @brief ROS action client node default ports
   * These ports are mandatory; child classes must include them if they override this method
   */
  static PortsList providedPorts()
  {
    return { InputPort<std::string>("action_name", "name of the ROS action client"),
             InputPort<double>("server_timeout", 5.0, "timeout to connect to server (seconds)") };
  }

  /**
   * @brief Required by a BT action. Make sure to cancel the ROS action if it is still running.
   */
  void halt() override
  {
    if (isActive())
    {
      action_client_->cancelGoal();
    }
    onFinished();
    setStatus(NodeStatus::IDLE);
  }

protected:
  std::string action_name_;
  typename std::unique_ptr<actionlib::SimpleActionClient<ActionT>> action_client_;

  /**
   * @brief To implement by the child class to provide a goal
   * It will be called on every tick, so children should return std::nullopt
   * after the first call unless they want to preempt the currently running goal
   * @return Optional new goal
   */
  virtual std::optional<GoalType> getGoal() = 0;

  /**
   * @brief The main override required by a BT action
   * @return NodeStatus Status of tick execution
   */
  inline NodeStatus tick() final
  {
    onTick();

    if (!action_client_)
    {
      if (!getInput("action_name", action_name_))
      {
        ROS_ERROR_NAMED(LOGNAME, "action_name is not defined");
        return NodeStatus::FAILURE;
      }

      double server_timeout;
      getInput("server_timeout", server_timeout);
      if (!createActionClient(server_timeout))
      {
        return NodeStatus::FAILURE;
      }
    }

    if (auto goal = getGoal(); goal)
    {
      if (!action_client_->isServerConnected())
      {
        ROS_ERROR_STREAM(LOGNAME << ": Action server " << action_name_ << " not connected");
        onServerUnavailable();
        return NodeStatus::FAILURE;
      }

      setStatus(NodeStatus::RUNNING);
      action_client_->sendGoal(*goal, {}, {}, boost::bind(&RosActionNode::feedbackCb, this, _1));
    }
    else if (status() == NodeStatus::IDLE)
    {
      ROS_ERROR_NAMED(LOGNAME, "No goal provided. Exiting with failure.");
      return NodeStatus::FAILURE;
    }

    auto action_state = action_client_->getState();
    if (action_state == actionlib::SimpleClientGoalState::PENDING ||
        action_state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      return NodeStatus::RUNNING;
    }
    else if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      onFinished();
      return onSucceeded(action_client_->getResult());
    }
    else if (action_state == actionlib::SimpleClientGoalState::ABORTED)
    {
      onFinished();
      return onAborted(action_client_->getResult());
    }
    else if (action_state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      onFinished();
      return onCanceled();
    }
    else if (action_state == actionlib::SimpleClientGoalState::REJECTED)
    {
      return onRejected();
    }
    else
    {
      ROS_ERROR_STREAM(LOGNAME << ": Action server " << action_name_ << " returned unknown state");
      return NodeStatus::FAILURE;
    }
  }
};

}  // namespace BT
