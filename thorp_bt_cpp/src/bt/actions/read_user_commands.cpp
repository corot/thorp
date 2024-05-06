#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_rviz_plugins/EusCommand.h>

#include <thorp_toolkit/visualization.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
class ReadUserCommands : public BT::SyncActionNode
{
public:
  ReadUserCommands(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
    ros::NodeHandle nh;
    valid_cmds_ = nh.param("object_manip_user_commands/valid_commands", valid_cmds_);
    cmd_pub_ = nh.advertise<jsk_rviz_plugins::OverlayText>("rviz/user_command", 1);
    cmd_srv_ = nh.advertiseService("eus_command", &ReadUserCommands::userCommandCB, this);
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("user_command") };
  }

private:
  ros::Publisher cmd_pub_;
  ros::ServiceServer cmd_srv_;
  std::string user_command_;
  std::vector<std::string> valid_cmds_;

  BT::NodeStatus tick() override
  {
    if (!user_command_.empty())
    {
      setOutput("user_command", user_command_);
      user_command_.clear();
    }
    return BT::NodeStatus::SUCCESS;
  }

  bool userCommandCB(jsk_rviz_plugins::EusCommand::Request  &request,
                     jsk_rviz_plugins::EusCommand::Response &response)
  {
    constexpr auto TOP_OFFSET = 20;
    constexpr auto TEXT_SIZE = 12;

    user_command_ = request.command;
    if (user_command_ == "exit")
    {
      std::string exit_msg = "Shutting down app";
      ROS_WARN_STREAM_NAMED(name(), exit_msg);
      cmd_pub_.publish(ttk::Visualization::createOverlayText(TOP_OFFSET, ttk::namedColor("blue"), exit_msg, TEXT_SIZE));
    }
    else if (auto it = std::find(valid_cmds_.begin(), valid_cmds_.end(), user_command_); it == valid_cmds_.end())
    {
      std::string err_msg = user_command_ + " command not supported";
      ROS_ERROR_STREAM_NAMED(name(), err_msg);
      cmd_pub_.publish(ttk::Visualization::createOverlayText(TOP_OFFSET, ttk::namedColor("red"), err_msg, TEXT_SIZE));
    }
    else
    {
      std::string cmd_msg = "Received command: " + user_command_;
      ROS_INFO_STREAM_NAMED(name(), cmd_msg);
      cmd_pub_.publish(ttk::Visualization::createOverlayText(TOP_OFFSET, ttk::namedColor("white"), cmd_msg, TEXT_SIZE));
    }

    return true;
  }

  BT_REGISTER_NODE(ReadUserCommands);
};
}  // namespace thorp::bt::actions
