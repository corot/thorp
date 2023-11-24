#include "thorp_bt_cpp/bt_nodes.hpp"

// bt actions

// navigation
#include "thorp_bt_cpp/bt/actions/go_to_pose.hpp"
#include "thorp_bt_cpp/bt/actions/smooth_path.hpp"
#include "thorp_bt_cpp/bt/actions/get_robot_pose.hpp"
#include "thorp_bt_cpp/bt/actions/track_progress.hpp"
#include "thorp_bt_cpp/bt/actions/clear_costmaps.hpp"
#include "thorp_bt_cpp/bt/actions/get_path_action.hpp"
#include "thorp_bt_cpp/bt/actions/exe_path_action.hpp"
#include "thorp_bt_cpp/bt/actions/recovery_action.hpp"
#include "thorp_bt_cpp/bt/actions/create_pose_stamped.hpp"

// exploration
#include "thorp_bt_cpp/bt/actions/segment_rooms.hpp"
#include "thorp_bt_cpp/bt/actions/plan_room_sequence.hpp"
#include "thorp_bt_cpp/bt/actions/plan_room_exploration.hpp"

// general
#include "thorp_bt_cpp/bt/actions/use_named_config.hpp"
#include "thorp_bt_cpp/bt/actions/set_blackboard.hpp"
#include "thorp_bt_cpp/bt/actions/list_slicing.hpp"
#include "thorp_bt_cpp/bt/actions/push_to_list.hpp"
#include "thorp_bt_cpp/bt/actions/pop_from_list.hpp"
#include "thorp_bt_cpp/bt/actions/get_list_front.hpp"

// bt conditions
#include "thorp_bt_cpp/bt/conditions/is_bool_true.hpp"
#include "thorp_bt_cpp/bt/conditions/is_list_empty.hpp"

// bt controls
#include "thorp_bt_cpp/bt/controls/super_reactive_sequence.hpp"

// bt decorators
#include "thorp_bt_cpp/bt/decorators/store_result.hpp"

// bt tools
#include <behaviortree_cpp_v3/xml_parsing.h>

#include <iostream>
#include <fstream>
#include <string>

namespace thorp::bt
{

void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh, const std::string& nodes_file_path)
{
  // actions
  factory.registerNodeType<bt::actions::GetRobotPose>("GetRobotPose");
  factory.registerNodeType<bt::actions::TrackProgress>("TrackProgress");
  factory.registerNodeType<bt::actions::CreatePoseStamped>("CreatePoseStamped");

  BT::RegisterRosService<bt::actions::SmoothPath>(factory, "SmoothPath", nh);
  BT::RegisterRosService<bt::actions::ClearCostmaps>(factory, "ClearCostmaps", nh);
  BT::RegisterSimpleActionClient<bt::actions::GoToPose>(factory, "GoToPose");
  BT::RegisterSimpleActionClient<bt::actions::GetPathAction>(factory, "GetPathAction");
  BT::RegisterSimpleActionClient<bt::actions::ExePathAction>(factory, "ExePathAction");
  BT::RegisterSimpleActionClient<bt::actions::RecoveryAction>(factory, "RecoveryAction");
  BT::RegisterSimpleActionClient<bt::actions::SegmentRooms>(factory, "SegmentRooms");
  BT::RegisterSimpleActionClient<bt::actions::PlanRoomSequence>(factory, "PlanRoomSequence");
  BT::RegisterSimpleActionClient<bt::actions::PlanRoomExploration>(factory, "PlanRoomExploration");

  factory.registerNodeType<bt::actions::UseNamedConfig>("UseNamedConfig");
  factory.registerNodeType<bt::actions::GetListFront<geometry_msgs::PoseStamped>>("GetPoseListFront");
  factory.registerNodeType<bt::actions::PushToList<geometry_msgs::PoseStamped>>("PushPoseToList");
  factory.registerNodeType<bt::actions::ListSlicing<geometry_msgs::PoseStamped>>("PoseListSlicing");
  factory.registerNodeType<bt::actions::PopFromList<uint32_t>>("PopUInt32FromList");
  factory.registerNodeType<bt::actions::SetBlackboard<bool>>("SetBool");
  factory.registerNodeType<bt::actions::SetBlackboard<double>>("SetDouble");
  factory.registerNodeType<bt::actions::SetBlackboard<unsigned int>>("SetUnsignedInt");

  // conditions
  factory.registerNodeType<bt::conditions::IsBoolTrue>("IsBoolTrue");
  factory.registerNodeType<bt::conditions::IsListEmpty<uint32_t>>("IsListEmpty");

  // controls
  factory.registerNodeType<bt::controls::SuperReactiveSequence>("SuperReactiveSequence");

  // dump to an XML file to load on Groot
  std::ofstream ofs;
  ofs.open(nodes_file_path,
           std::ios::out |    // output file stream
           std::ios::trunc);  // truncate content
  if (ofs)
  {
    ofs << BT::writeTreeNodesModelXML(factory) << std::endl;
    ofs.close();
  }
  else
  {
    ROS_ERROR_STREAM("Unable to open file to write node models file: " << nodes_file_path);
  }
}

}  // namespace thorp::bt
