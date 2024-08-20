#pragma once

// behaviortree_cpp
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

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

  RosActionNode(const std::string& xml_tag_name, const NodeConfig& conf) : ActionNodeBase(xml_tag_name, conf)
  {
  }

private:
  static constexpr char LOGNAME[] = "RosActionNode";

  inline NodeStatus waitForServer(ros::Time created_time, ros::Duration timeout)
  {
    // Return RUNNING until the server comes alive (return SUCCESS) or timeout occurs (return FAILURE)
    if (action_client_->waitForServer(ros::Duration(1e-3)))
    {
      ROS_INFO_STREAM(LOGNAME << ": Action server " << action_name_ << " found");
      server_connected_ = true;
      return NodeStatus::SUCCESS;
    }
    else if (timeout.isZero() || ros::Time::now() - created_time < timeout)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, LOGNAME << ": Waiting for action server " << action_name_);
      return NodeStatus::RUNNING;
    }
    else
    {
      ROS_ERROR_STREAM(LOGNAME << ": Action server " << action_name_ << " not found");
      onServerUnavailable();
      return NodeStatus::FAILURE;
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
  }

protected:
  bool server_connected_ = false;
  ros::Duration server_timeout_;
  std::string action_name_;
  typename std::unique_ptr<actionlib::SimpleActionClient<ActionT>> action_client_;

  bool goal_updated_ = false;

  /**
   * @brief To implement by the child class to provide a goal.
   * It will be called either if the action is idle or the child sets the flag goal_updated_ to true.
   * @return A new goal
   */
  virtual GoalType getGoal() = 0;

  /**
   * @brief The main override required by a BT action
   * @return NodeStatus Status of tick execution
   */
  inline NodeStatus tick() final
  {
    onTick();

    static ros::Time created_time;  // persistent through executions to control if we hit server_timeout_

    if (!action_client_)
    {
      if (double timeout; getInput<double>("server_timeout", timeout))
      {
        server_timeout_.fromSec(timeout);
      }
      if (!getInput("action_name", action_name_))
      {
        ROS_ERROR_NAMED(LOGNAME, "action_name is not defined");
        return NodeStatus::FAILURE;
      }
      action_client_ = std::make_unique<actionlib::SimpleActionClient<ActionT>>(action_name_, true);
      created_time = ros::Time::now();
    }

    if (!server_connected_)
    {
      if (NodeStatus status = waitForServer(created_time, server_timeout_); status != NodeStatus::SUCCESS)
      {
        return status;
      }
      resetStatus();  // trigger sending the first goal for children not setting goal_updated_
    }

    if (status() == NodeStatus::IDLE || goal_updated_)
    {
      goal_updated_ = false;
      if (!action_client_->isServerConnected())
      {
        ROS_ERROR_STREAM(LOGNAME << ": Action server " << action_name_ << " not connected");
        onServerUnavailable();
        return NodeStatus::FAILURE;
      }
      auto goal = getGoal();
      action_client_->sendGoal(goal, {}, {}, boost::bind(&RosActionNode::feedbackCb, this, _1));
    }

    auto action_state = action_client_->getState();
    switch (action_state.state_)
    {
      case actionlib::SimpleClientGoalState::PENDING:
      case actionlib::SimpleClientGoalState::ACTIVE:
        return NodeStatus::RUNNING;
      case actionlib::SimpleClientGoalState::SUCCEEDED:
        onFinished();
        return onSucceeded(action_client_->getResult());
      case actionlib::SimpleClientGoalState::ABORTED:
        onFinished();
        return onAborted(action_client_->getResult());
      case actionlib::SimpleClientGoalState::PREEMPTED:
        onFinished();
        return onCanceled();
      case actionlib::SimpleClientGoalState::REJECTED:
        return onRejected();
      default:
        ROS_ERROR_STREAM(LOGNAME << ": Unexpected " << action_name_ << " action state: " << action_state.toString());
        return NodeStatus::FAILURE;
    }
  }
};

}  // namespace BT
