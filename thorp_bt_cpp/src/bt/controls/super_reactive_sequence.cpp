#include <behaviortree_cpp_v3/control_node.h>

#include "thorp_bt_cpp/node_register.hpp"

namespace thorp::bt::controls
{
/** Adapted from BT::ReactiveSequence to allow for multiple asynchronous children. */
class SuperReactiveSequence : public BT::ControlNode
{
public:
  SuperReactiveSequence(const std::string& name, const BT::NodeConfiguration& config) : ControlNode(name, config)
  {
  }

private:
  virtual BT::NodeStatus tick() override
  {
    size_t success_count = 0;
    size_t running_count = 0;

    for (size_t index = 0; index < childrenCount(); index++)
    {
      TreeNode* current_child_node = children_nodes_[index];
      const BT::NodeStatus child_status = current_child_node->executeTick();

      switch (child_status)
      {
        case BT::NodeStatus::RUNNING:
        {
          running_count++;

          // this is the only difference to the original code:
          // halt prev children too, not just future ones
          for (size_t i = 0; i < childrenCount(); i++)
          {
            if (i != index)
            {
              haltChild(i);
            }
          }

          return BT::NodeStatus::RUNNING;
        }

        case BT::NodeStatus::FAILURE:
        {
          resetChildren();
          return BT::NodeStatus::FAILURE;
        }
        case BT::NodeStatus::SUCCESS:
        {
          success_count++;
        }
        break;

        case BT::NodeStatus::IDLE:
        {
          throw BT::LogicError("A child node must never return IDLE");
        }
      }  // end switch
    }    // end for

    if (success_count == childrenCount())
    {
      resetChildren();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT_REGISTER_NODE(SuperReactiveSequence);
};
}  // namespace thorp::bt::controls
