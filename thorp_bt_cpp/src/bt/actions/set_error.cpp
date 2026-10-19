#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/errors.hpp"
#include "thorp_bt_cpp/node_common.hpp"

#include <thorp_msgs/msg/thorp_error.hpp>

namespace thorp::bt::actions
{
/**
 * State an error, for a tree to say what a failed check means for it, as a pickup whose gripper closed empty: a code,
 * named as ThorpError or MoveItErrorCodes define it, and what failed in a line.
 */
class SetError : public BT::SyncActionNode
{
public:
  // A misspelled code fails when the tree is built, rather than when its branch runs
  SetError(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config), code_(errorCode(requireInput<std::string>(*this, "code")))
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("code", "error code name, as ThorpError or MoveItErrorCodes define it"),
             BT::InputPort<std::string>("text", "", "what failed, in a line"),
             BT::OutputPort<thorp_msgs::msg::ThorpError>("error") };
  }

private:
  BT::NodeStatus tick() override
  {
    setOutput("error", makeError(code_, getInput<std::string>("text").value_or("")));
    return BT::NodeStatus::SUCCESS;
  }

  const int32_t code_;

  BT_REGISTER_NODE(SetError);
};

}  // namespace thorp::bt::actions
