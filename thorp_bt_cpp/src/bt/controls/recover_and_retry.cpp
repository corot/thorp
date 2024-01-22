#include <behaviortree_cpp_v3/control_node.h>

#include "thorp_bt_cpp/node_register.hpp"

namespace thorp::bt::controls
{
/**
 * @brief The RecoverAndRetry has one main task child and one or more recovery children.
 * Returns SUCCESS if and only if the first child returns SUCCESS.
 *
 * - If the first child returns FAILURE, we run the subsequent children in sequence,
 *   retrying the main task whenever one recovery child succeeds.
 *
 * - If any child returns RUNNING, this node returns RUNNING.
 *
 * - If the main task returns FAILURE after having executed all recovery children,
 *   we reset the control node and return FAILURE.
 */
class RecoverAndRetry : public BT::ControlNode
{
public:
  RecoverAndRetry(const std::string& name, const BT::NodeConfiguration& config) : ControlNode(name, config)
  {
  }

private:
  unsigned int current_child_idx_ = 0;
  unsigned int last_recovery_run_ = 0;

  BT::NodeStatus tick() override
  {
    const unsigned children_count = children_nodes_.size();
    const unsigned recovery_count = children_count - 1;

    while (current_child_idx_ < children_count)
    {
      TreeNode* child_node = children_nodes_[current_child_idx_];
      const BT::NodeStatus child_status = child_node->executeTick();

      if (current_child_idx_ == 0)
      {
        switch (child_status)
        {
          case BT::NodeStatus::SUCCESS:
          {
            // reset node and return success when first child returns success
            halt();
            return BT::NodeStatus::SUCCESS;
          }

          case BT::NodeStatus::FAILURE:
          {
            if (last_recovery_run_ < recovery_count)
            {
              // run next recovery child
              current_child_idx_ = last_recovery_run_ + 1;
              break;
            }
            else
            {
              // reset node and return failure if we have no more recovery children to try
              halt();
              return BT::NodeStatus::FAILURE;
            }
          }

          case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

          default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
        }  // end switch
      }
      else if (current_child_idx_ > 0)
      {
        switch (child_status)
        {
          case BT::NodeStatus::SUCCESS:
          {
            // retry main task
            last_recovery_run_ = current_child_idx_;
            current_child_idx_ = 0;
          }
          break;

          case BT::NodeStatus::FAILURE:
          {
            // run next recovery child
            last_recovery_run_ = current_child_idx_;
            current_child_idx_++;
          }

          case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

          default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
        }  // end switch
      }
    }  // end while loop

    // reset node and return failure
    halt();
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override
  {
    ControlNode::halt();
    current_child_idx_ = 0;
    last_recovery_run_ = 0;
  }

  BT_REGISTER_NODE(RecoverAndRetry);
};
}  // namespace thorp::bt::controls
