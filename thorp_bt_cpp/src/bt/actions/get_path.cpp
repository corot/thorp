#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/action_client_node.hpp"

#include <mbf_msgs/GetPathAction.h>

namespace thorp::bt::actions
{
class GetPath : public BT::SimpleActionClientNode<mbf_msgs::GetPathAction>
{
public:
  GetPath(const std::string& name, const BT::NodeConfiguration& config) : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = BT::SimpleActionClientNode<ActionType>::providedPorts();
    ports["action_name"].setDefaultValue("move_base_flex/get_path");
    ports.insert({ BT::InputPort<std::string>("planner"),                     //
                   BT::InputPort<geometry_msgs::PoseStamped>("target_pose"),  //
                   BT::OutputPort<unsigned int>("error"),                     //
                   BT::OutputPort<std::optional<FeedbackType>>("feedback") });
    return ports;
  }

  bool setGoal(GoalType& goal) override
  {
    goal.planner = *getInput<std::string>("planner");
    goal.target_pose = *getInput<geometry_msgs::PoseStamped>("target_pose");
    return true;
  }

  void onFeedback(const FeedbackConstPtr& feedback) override
  {
    setOutput("feedback", std::make_optional<FeedbackType>(*feedback));
  }

  BT::NodeStatus onAborted(const ResultConstPtr& res) override
  {
    ROS_ERROR_NAMED(name(), "Error %d: %s", res->outcome, res->message.c_str());
    setOutput("error", res->outcome);

    return BT::NodeStatus::FAILURE;
  }

private:
  BT_REGISTER_NODE(GetPath);
};
}  // namespace thorp::bt::actions
