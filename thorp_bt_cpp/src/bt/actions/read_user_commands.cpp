#include <algorithm>
#include <mutex>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <std_msgs/msg/string.hpp>

#include <thorp_toolkit/visualization.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Output the last command received on the user_command topic, as RViz's user commands panel publishes them, if
 * any since the previous tick. Echo it, or the error if the app doesn't support it, as an RViz overlay text.
 */
class ReadUserCommands : public BT::SyncActionNode
{
public:
  ReadUserCommands(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
    auto node = rosNode(*this);
    feedback_pub_ = node->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("rviz/user_command", 1);
    command_sub_ = node->create_subscription<std_msgs::msg::String>(
        "user_command", 1, [this](const std_msgs::msg::String::ConstSharedPtr msg) { userCommandCB(msg->data); });
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<std::string>>("valid_commands", "Commands the app supports, ';' separated"),
             BT::OutputPort<std::string>("user_command") };
  }

private:
  rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr feedback_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  std::mutex mutex_;  // bt_server receives commands on another thread than the one ticking the tree
  std::string user_command_;

  BT::NodeStatus tick() override
  {
    std::string user_command;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      user_command.swap(user_command_);
    }
    if (user_command.empty())
    {
      return BT::NodeStatus::SUCCESS;
    }

    constexpr auto TOP_OFFSET = 20;
    constexpr auto TEXT_SIZE = 12;
    const auto valid_commands = requireInput<std::vector<std::string>>(*this, "valid_commands");
    if (std::find(valid_commands.begin(), valid_commands.end(), user_command) == valid_commands.end())
    {
      const std::string error = user_command + " command not supported";
      RCLCPP_ERROR_STREAM(logger(*this), error);
      feedback_pub_->publish(
          ttk::Visualization::createOverlayText(TOP_OFFSET, ttk::namedColor("red"), error, TEXT_SIZE));
      return BT::NodeStatus::SUCCESS;
    }

    const std::string message = user_command == "exit" ? "Shutting down app" : "Received command: " + user_command;
    RCLCPP_INFO_STREAM(logger(*this), message);
    feedback_pub_->publish(ttk::Visualization::createOverlayText(
        TOP_OFFSET, ttk::namedColor(user_command == "exit" ? "blue" : "white"), message, TEXT_SIZE));
    setOutput("user_command", user_command);
    return BT::NodeStatus::SUCCESS;
  }

  void userCommandCB(const std::string& command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    user_command_ = command;
  }

  BT_REGISTER_NODE(ReadUserCommands);
};
}  // namespace thorp::bt::actions
