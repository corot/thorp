#include <behaviortree_cpp_v3/decorator_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_msgs/PickupLocation.h>

namespace thorp::bt::decorators
{
/**
 * Loops over a list, popping a new element whenever the child returns SUCCESS.
 * Finishes once the list is empty or the child returns FAILURE.
 *
 * Adapted from BT::ConsumeQueue.
 *
 * @param[in, out] list    Input list
 * @param[out]     element Popped elements
 *
 * @return  SUCCESS if the list is empty
 *          RUNNING if the child is running
 *          FAILURE if the child fails
 */
template <typename T>
class ForEach : public BT::DecoratorNode
{
public:
  ForEach(const std::string& name, const BT::NodeConfiguration& config)
    : BT::DecoratorNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::BidirectionalPort<std::vector<T>>("list"),  //
             BT::OutputPort<T>("element") };
  }

  BT::NodeStatus tick() override
  {
    if (running_child_)
    {
      BT::NodeStatus child_state = child_node_->executeTick();
      running_child_ = child_state == BT::NodeStatus::RUNNING;
      if (running_child_)
      {
        return BT::NodeStatus::RUNNING;
      }
      haltChild();
      if (child_state == BT::NodeStatus::FAILURE)
      {
        return BT::NodeStatus::FAILURE;
      }
    }

    auto list = *getInput<std::vector<T>>("list");
    while (!list.empty())
    {
      setStatus(BT::NodeStatus::RUNNING);

      setOutput("element", list.front());
      list.erase(list.begin());
      setOutput("list", list);
      ROS_DEBUG_STREAM(name() << ":\tnew size is " << list.size());

      BT::NodeStatus child_state = child_node_->executeTick();
      running_child_ = child_state == BT::NodeStatus::RUNNING;
      if (running_child_)
      {
        return BT::NodeStatus::RUNNING;
      }
      else
      {
        haltChild();
        if (child_state == BT::NodeStatus::FAILURE)
        {
          return BT::NodeStatus::FAILURE;
        }
      }
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  bool running_child_ = false;

  static NodeRegister<ForEach<T>> reg_;
};

// Register a builder for each templated version this class
BT_REGISTER_TEMPLATE_NODE(ForEach<thorp_msgs::PickupLocation>, "ForEachPickupLocation");
}  // namespace thorp::bt::decorators
