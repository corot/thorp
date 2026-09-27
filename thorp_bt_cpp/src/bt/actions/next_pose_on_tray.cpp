#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/planning_scene.hpp"
#include "thorp_bt_cpp/tray.hpp"

namespace thorp::bt::actions
{
/**
 * Pose to place an object on the next free slot of the tray.
 *
 * @return  SUCCESS with the pose, FAILURE if the tray is full
 */
class NextPoseOnTray : public BT::SyncActionNode
{
public:
  NextPoseOnTray(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_on_tray") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto free_slots = Tray(rosNode(*this)).freeSlots(planningScene().getObjects());
    if (free_slots.empty())
    {
      RCLCPP_ERROR(logger(*this), "Tray is full");
      return BT::NodeStatus::FAILURE;
    }
    setOutput("pose_on_tray", free_slots.front());
    return BT::NodeStatus::SUCCESS;
  }

  BT_REGISTER_NODE(NextPoseOnTray);
};

}  // namespace thorp::bt::actions
