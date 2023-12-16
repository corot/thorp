#pragma once

// behaviortree_cpp
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// std
#include <optional>
#include <mutex>

// ros
#include <ros/ros.h>

namespace BT
{

/**
 * @brief A ROS subscriber action for BehaviorTree.CPP that subscribes to a topic until a message is received or a
 * timeout (sec).
 */
template <class SubscriberT>
class SubscriberNode : public BT::StatefulActionNode
{
public:
  using SubscriberType = SubscriberT;

  SubscriberNode(const std::string& name, const BT::NodeConfiguration& conf) : BT::StatefulActionNode(name, conf)
  {
  }

  SubscriberNode() = delete;

  virtual ~SubscriberNode() = default;

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("topic_name", "name of the ROS topic"),
             InputPort<double>("timeout", 1.0, "timeout to subscribe to topic (sec)"),
             InputPort<double>("max_delay", 0.0, "max delay wrt Time::now() (sec); ignored if negative") };
  }

public:
  inline NodeStatus onStart() final
  {
    start_time_ = ros::Time::now();

    getInput("topic_name", topic_);
    if (!topic_.empty())
    {
      onStarted();
      sub_ = ros::NodeHandle().subscribe(topic_, 1, &SubscriberNode::callback, this);
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "topic_name is empty");
      return NodeStatus::FAILURE;
    }

    double sec;
    getInput("timeout", sec);
    timeout_ = sec <= 0 ? ros::Duration(0) : ros::Duration(sec);

    double max_delay;
    getInput("max_delay", max_delay);
    max_delay_ = max_delay <= 0 ? ros::Duration(0) : ros::Duration(max_delay);

    return onRunning();
  }

  inline NodeStatus onRunning() final
  {
    BT::NodeStatus status;
    {
      // lock for async spinners
      std::lock_guard lock(async_spin_mtx_);
      if (nmsg_)
      {
        {
          std::lock_guard lock(msg_mtx_);
          msg_ = nmsg_.value();
        }

        if constexpr (ros::message_traits::HasHeader<SubscriberT>::value)
        {
          if (max_delay_ != ros::Duration(0) && ros::Time::now() - msg_.header.stamp > max_delay_)
          {
            ROS_DEBUG_STREAM_NAMED(LOGNAME, "Message received from topic "
                                                << topic_ << " is too old: " << ros::Time::now() - msg_.header.stamp
                                                << " > " << max_delay_);
            status = onMaxDelay();
          }
          else
          {
            status = onReceived();
          }
        }
        else
        {
          status = onReceived();
        }
        nmsg_.reset();
      }
      else
      {
        const auto action_elapsed_time = ros::Time::now() - start_time_;
        if (timeout_ != ros::Duration(0) && action_elapsed_time > timeout_)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME,
                                 "No message received in topic " << topic_ << " after " << action_elapsed_time);
          status = onTimeout();
        }
        else
        {
          ROS_DEBUG_STREAM_THROTTLE_NAMED(
              1.0, LOGNAME, "Waiting for message from topic " << topic_ << "; elapsed time: " << action_elapsed_time);
          return NodeStatus::RUNNING;
        }
      }
    }

    // sub_.shutdown();  onStart doesn't get called on subsequent executions!!!
    return status;
  }

  inline void onHalted() final
  {
    onHalt();
    {
      std::lock_guard lock(async_spin_mtx_);
      nmsg_.reset();
    }
    sub_.shutdown();
  }

protected:
  virtual void onStarted()
  {
  }

  virtual BT::NodeStatus onReceived()
  {
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus onTimeout()
  {
    return BT::NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onMaxDelay()
  {
    return BT::NodeStatus::FAILURE;
  }

  virtual void onHalt()
  {
  }
  std::string topic_;
  ros::Duration timeout_;
  ros::Duration max_delay_;

  /**
   * @brief get the reference to the message and the lock to the mutex. This is thread-safe if used correctly.
   * This does not return a copy of the message, but a reference to it. The lock is returned to avoid copying it.
   * Use it carefully, i.e., do not modify the message outside the lock scope.
   * Example of not to do:
   *
    SubscriberT& msgRef;
    {
      auto [lock, msg] = getMsg();
      msgRef = msg;
      // lock is destroyed here, msg is no longer protected by mutex
    }
   */
  std::pair<std::unique_lock<std::mutex>, SubscriberT&> getMsg()
  {
    std::unique_lock lock(msg_mtx_);
    return { std::move(lock), msg_ };
  }

  /**
   * @brief get a copy of the message (thread safe)
   * @return a copy of the message
   */
  SubscriberT getMsgCopy()
  {
    std::lock_guard lock(msg_mtx_);
    return msg_;
  }

private:
  static constexpr auto LOGNAME = "SubscriberNode";

  std::optional<SubscriberT> nmsg_;
  SubscriberT msg_;
  std::mutex async_spin_mtx_;
  std::mutex msg_mtx_;
  ros::Time start_time_;
  ros::Subscriber sub_;

  void callback(const SubscriberT& msg)
  {
    // lock for async spinners
    std::lock_guard lock(async_spin_mtx_);
    nmsg_ = msg;
  }
};

template <class DerivedT>
static void RegisterSubscriber(BT::BehaviorTreeFactory& factory, const std::string& registration_ID)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  { return std::make_unique<DerivedT>(name, config); };

  BT::TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = SubscriberNode<typename DerivedT::SubscriberType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
}

}  // namespace BT
