#include <map>
#include <mutex>

#include <tinyxml2.h>

#include <nav2_behavior_tree/bt_action_node.hpp>

#include "thorp_bt_cpp/node_common.hpp"

#include <moveit_msgs/action/move_group.hpp>
#include <std_msgs/msg/string.hpp>

namespace thorp::bt::actions
{
/**
 * Move the arm to one of the named configurations (group states) in the SRDF, through move_group.
 */
class SetArmConfig : public nav2_behavior_tree::BtActionNode<moveit_msgs::action::MoveGroup>
{
public:
  SetArmConfig(const std::string& name, const std::string& action_name, const BT::NodeConfig& config)
    : BtActionNode(name, action_name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>("configuration"),  //
                                BT::OutputPort<int>("error") });
  }

private:
  /** A named configuration: its group and the joint values */
  using NamedState = std::pair<std::string, std::map<std::string, double>>;

  void on_tick() override
  {
    const auto configuration = requireInput<std::string>(*this, "configuration");
    const auto& named_states = namedStates(node_);
    const auto state = named_states.find(configuration);
    if (state == named_states.end())
    {
      throw BT::RuntimeError(name(), ": unknown arm configuration ", configuration);
    }

    moveit_msgs::msg::Constraints constraints;
    for (const auto& [joint, value] : state->second.second)
    {
      moveit_msgs::msg::JointConstraint joint_constraint;
      joint_constraint.joint_name = joint;
      joint_constraint.position = value;
      joint_constraint.tolerance_above = joint_constraint.tolerance_below = 0.01;
      joint_constraint.weight = 1.0;
      constraints.joint_constraints.push_back(joint_constraint);
    }
    goal_ = moveit_msgs::action::MoveGroup::Goal();
    goal_.request.group_name = state->second.first;
    goal_.request.goal_constraints.push_back(constraints);
    // The OMPL configuration's default planner; unnamed, MoveIt warns as OMPL doesn't report it
    goal_.request.pipeline_id = "ompl";
    goal_.request.planner_id = "RRTConnect";
    // Workspace bounds only matter to planar or floating joints, that the arm lacks; MoveIt warns if there are none
    goal_.request.workspace_parameters.header.frame_id = "base_footprint";
    goal_.request.workspace_parameters.min_corner.x = goal_.request.workspace_parameters.min_corner.y =
        goal_.request.workspace_parameters.min_corner.z = -1.0;
    goal_.request.workspace_parameters.max_corner.x = goal_.request.workspace_parameters.max_corner.y =
        goal_.request.workspace_parameters.max_corner.z = 1.0;
    goal_.request.allowed_planning_time = 5.0;
    goal_.request.num_planning_attempts = 1;
    goal_.request.max_velocity_scaling_factor = 1.0;
    goal_.request.max_acceleration_scaling_factor = 1.0;
  }

  BT::NodeStatus on_success() override
  {
    if (result_.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      return on_aborted();
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_aborted() override
  {
    RCLCPP_ERROR(logger(*this), "Error %d moving arm to %s", result_.result->error_code.val,
                 requireInput<std::string>(*this, "configuration").c_str());
    setOutput<int>("error", result_.result->error_code.val);
    return BT::NodeStatus::FAILURE;
  }

  /**
   * The SRDF group states, read once from move_group's robot_description_semantic topic.
   */
  static const std::map<std::string, NamedState>& namedStates(const rclcpp::Node::SharedPtr& node)
  {
    static std::map<std::string, NamedState> named_states;
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    if (!named_states.empty())
    {
      return named_states;
    }

    std::string srdf;
    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(callback_group, node->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;
    auto sub = node->create_subscription<std_msgs::msg::String>(
        "robot_description_semantic", rclcpp::QoS(1).transient_local(),
        [&srdf](const std_msgs::msg::String::ConstSharedPtr msg) { srdf = msg->data; }, options);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (srdf.empty() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some(std::chrono::milliseconds(100));
    }
    if (srdf.empty())
    {
      throw BT::RuntimeError("SetArmConfig: no robot_description_semantic received");
    }

    tinyxml2::XMLDocument doc;
    doc.Parse(srdf.c_str());
    for (auto* state = doc.RootElement()->FirstChildElement("group_state"); state;
         state = state->NextSiblingElement("group_state"))
    {
      NamedState& named_state = named_states[state->Attribute("name")];
      named_state.first = state->Attribute("group");
      for (auto* joint = state->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint"))
      {
        named_state.second[joint->Attribute("name")] = joint->DoubleAttribute("value");
      }
    }
    return named_states;
  }

  BT_REGISTER_ACTION_NODE(SetArmConfig, "move_action");
};

}  // namespace thorp::bt::actions
