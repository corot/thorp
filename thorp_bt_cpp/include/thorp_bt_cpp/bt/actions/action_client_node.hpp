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
// TODO PR this to upstream
template <class ActionT>
class SimpleActionClientNode : public BT::ActionNodeBase
{
protected:
  using GoalType = typename ActionT::_action_goal_type::_goal_type;
  using ResultType = typename ActionT::_action_result_type::_result_type;
  using FeedbackType = typename ActionT::_action_feedback_type::_feedback_type;
  typedef boost::shared_ptr<const FeedbackType> FeedbackConstPtr;
  typedef boost::shared_ptr<const ResultType> ResultConstPtr;

  SimpleActionClientNode(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
  {
  }

private:
  static constexpr char LOGNAME[] = "SimpleActionClientNode";

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
    feedback_ = feedback;
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
   * @brief Function to perform some user-defined operation if a new goal is received. If it returns True, the new goal
   * is sent to the server.
   */
  virtual bool onNewGoalReceived(const GoalType& goal)
  {
    return true;
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
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus onSuccess(const ResultConstPtr& res)
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation whe the action is aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   */
  virtual BT::NodeStatus onAborted(const ResultConstPtr& res)
  {
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus onCancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation when the goal is rejected by server.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   */
  virtual BT::NodeStatus onRejected()
  {
    return BT::NodeStatus::FAILURE;
  }

public:
  using ActionType = ActionT;

  SimpleActionClientNode() = delete;
  virtual ~SimpleActionClientNode() = default;

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("action_name", "name of the ROS action client"),
             InputPort<double>("server_timeout", 5.0, "timeout to connect to server (seconds)"),
             InputPort<GoalType>("goal", "goal to send to the action server") };
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
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  std::string action_name_;
  typename std::unique_ptr<actionlib::SimpleActionClient<ActionT>> action_client_;
  GoalType goal_;
  FeedbackConstPtr feedback_;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  inline BT::NodeStatus tick() final
  {
    if (!action_client_)
    {
      if (!getInput("action_name", action_name_))
      {
        ROS_ERROR_NAMED(LOGNAME, "action_name is not defined");
        return BT::NodeStatus::FAILURE;
      }

      double server_timeout;
      getInput("server_timeout", server_timeout);
      if (!createActionClient(server_timeout))
      {
        return BT::NodeStatus::FAILURE;
      }
    }

    if (status() == BT::NodeStatus::IDLE)
    {
      if (!getInput("goal", goal_))
      {
        ROS_WARN_NAMED(LOGNAME, "goal is not defined. Exiting with success.");
        return BT::NodeStatus::SUCCESS;
      }

      setStatus(BT::NodeStatus::RUNNING);

      if (!action_client_->isServerConnected())
      {
        ROS_ERROR_STREAM(LOGNAME << ": Action server " << action_name_ << " not connected");
        onServerUnavailable();
        return BT::NodeStatus::FAILURE;
      }

      onTick();
      feedback_.reset();  // TODO this is a fix needed
      action_client_->sendGoal(goal_, {}, {}, boost::bind(&SimpleActionClientNode::feedbackCb, this, _1));
    }

    else
    {
      onTick();
      GoalType new_goal;
      if (getInput("goal", new_goal))
      {
        if (new_goal != goal_)
        {
          if (onNewGoalReceived(new_goal))
          {
            goal_ = new_goal;
            feedback_.reset();  // TODO this is a fix needed
            action_client_->sendGoal(goal_, {}, {}, boost::bind(&SimpleActionClientNode::feedbackCb, this, _1));
          }
          else
          {
            ROS_INFO_NAMED(LOGNAME, "New goal received but not sent to server");
          }
        }
      }
    }

    auto action_state = action_client_->getState();

    if (action_state == actionlib::SimpleClientGoalState::PENDING ||
        action_state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      if (feedback_)
      {
        onFeedback(feedback_);
      }
      return NodeStatus::RUNNING;
    }
    else if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return onSuccess(action_client_->getResult());
    }
    else if (action_state == actionlib::SimpleClientGoalState::ABORTED)
    {
      return onAborted(action_client_->getResult());
    }
    else if (action_state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      return onCancelled();
    }
    else if (action_state == actionlib::SimpleClientGoalState::REJECTED)
    {
      return onRejected();
    }
    else
    {
      ROS_ERROR_STREAM(LOGNAME << ": Action server " << action_name_ << " returned unknown state");
      return BT::NodeStatus::FAILURE;
    }
  }
};

template <class DerivedT>
static void RegisterSimpleActionClient(BT::BehaviorTreeFactory& factory, const std::string& registration_ID)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  { return std::make_unique<DerivedT>(name, config); };

  BT::TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = SimpleActionClientNode<typename DerivedT::ActionType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
}

}  // namespace BT
