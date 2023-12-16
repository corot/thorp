#include "thorp_bt_cpp/bt_nodes.hpp"

// bt actions

// navigation
#include "thorp_bt_cpp/bt/actions/go_to_pose.hpp"
#include "thorp_bt_cpp/bt/actions/follow_pose.hpp"
#include "thorp_bt_cpp/bt/actions/smooth_path.hpp"
#include "thorp_bt_cpp/bt/actions/get_robot_pose.hpp"
#include "thorp_bt_cpp/bt/actions/track_progress.hpp"
#include "thorp_bt_cpp/bt/actions/clear_costmaps.hpp"
#include "thorp_bt_cpp/bt/actions/get_path_action.hpp"
#include "thorp_bt_cpp/bt/actions/exe_path_action.hpp"
#include "thorp_bt_cpp/bt/actions/recovery_action.hpp"
#include "thorp_bt_cpp/bt/actions/create_pose_stamped.hpp"

// perception
#include "thorp_bt_cpp/bt/actions/monitor_objects.hpp"

// exploration
#include "thorp_bt_cpp/bt/actions/segment_rooms.hpp"
#include "thorp_bt_cpp/bt/actions/plan_room_sequence.hpp"
#include "thorp_bt_cpp/bt/actions/plan_room_exploration.hpp"

// manipulation
#include "thorp_bt_cpp/bt/actions/cannon_control.hpp"

// general
#include "thorp_bt_cpp/bt/actions/use_named_config.hpp"
#include "thorp_bt_cpp/bt/actions/set_blackboard.hpp"
#include "thorp_bt_cpp/bt/actions/list_slicing.hpp"
#include "thorp_bt_cpp/bt/actions/get_from_list.hpp"
#include "thorp_bt_cpp/bt/actions/push_to_list.hpp"
#include "thorp_bt_cpp/bt/actions/pop_from_list.hpp"
#include "thorp_bt_cpp/bt/actions/get_list_front.hpp"
#include "thorp_bt_cpp/bt/actions/clear_list.hpp"
#include "thorp_bt_cpp/bt/actions/sleep.hpp"

// bt conditions
#include "thorp_bt_cpp/bt/conditions/is_bool_true.hpp"
#include "thorp_bt_cpp/bt/conditions/is_list_empty.hpp"
#include "thorp_bt_cpp/bt/conditions/in_blackboard.hpp"
#include "thorp_bt_cpp/bt/conditions/target_reachable.hpp"

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
  using geometry_msgs::PoseStamped;

  // actions
  factory.registerNodeType<bt::actions::GetRobotPose>("GetRobotPose");
  factory.registerNodeType<bt::actions::TrackProgress>("TrackProgress");
  factory.registerNodeType<bt::actions::CreatePoseStamped>("CreatePoseStamped");
  factory.registerNodeType<bt::actions::UseNamedConfig>("UseNamedConfig");
  factory.registerNodeType<bt::actions::Sleep>("Sleep");

  BT::RegisterSubscriber<bt::actions::MonitorObjects>(factory, "MonitorObjects");
  BT::RegisterRosService<bt::actions::SmoothPath>(factory, "SmoothPath");
  BT::RegisterRosService<bt::actions::ClearCostmaps>(factory, "ClearCostmaps");
  BT::RegisterRosService<bt::actions::AimCannon>(factory, "AimCannon");
  BT::RegisterRosService<bt::actions::TiltCannon>(factory, "TiltCannon");
  BT::RegisterRosService<bt::actions::FireCannon>(factory, "FireCannon");
  BT::RegisterSimpleActionClient<bt::actions::GoToPose>(factory, "GoToPose");
  BT::RegisterSimpleActionClient<bt::actions::FollowPose>(factory, "FollowPose");
  BT::RegisterSimpleActionClient<bt::actions::GetPathAction>(factory, "GetPathAction");
  BT::RegisterSimpleActionClient<bt::actions::ExePathAction>(factory, "ExePathAction");
  BT::RegisterSimpleActionClient<bt::actions::RecoveryAction>(factory, "RecoveryAction");
  BT::RegisterSimpleActionClient<bt::actions::SegmentRooms>(factory, "SegmentRooms");
  BT::RegisterSimpleActionClient<bt::actions::PlanRoomSequence>(factory, "PlanRoomSequence");
  BT::RegisterSimpleActionClient<bt::actions::PlanRoomExploration>(factory, "PlanRoomExploration");

  // Blackboard access
  factory.registerNodeType<bt::actions::GetListFront<PoseStamped>>("GetPoseListFront");
  factory.registerNodeType<bt::actions::GetFromList<uint32_t>>("GetUIntFromList");
  factory.registerNodeType<bt::actions::PopFromList<uint32_t>>("PopUIntFromList");
  factory.registerNodeType<bt::actions::PushToList<PoseStamped>>("PushPoseToList");
  factory.registerNodeType<bt::actions::ListSlicing<PoseStamped>>("PoseListSlicing");
  factory.registerNodeType<bt::actions::PopFromList<PoseStamped>>("PopPoseFromList");
  factory.registerNodeType<bt::actions::ClearList<PoseStamped>>("ClearPoseList");
  factory.registerNodeType<bt::actions::SetBlackboard<bool>>("SetBool");
  factory.registerNodeType<bt::actions::SetBlackboard<double>>("SetDouble");
  factory.registerNodeType<bt::actions::SetBlackboard<uint32_t>>("SetUnsignedInt");

  // conditions
  factory.registerNodeType<bt::conditions::TargetReachable>("TargetReachable");
  factory.registerNodeType<bt::conditions::IsBoolTrue>("IsBoolTrue");
  factory.registerNodeType<bt::conditions::IsListEmpty<uint32_t>>("IsUIntListEmpty");
  factory.registerNodeType<bt::conditions::IsListEmpty<PoseStamped>>("IsPoseListEmpty");
  factory.registerNodeType<bt::conditions::InBlackboard<PoseStamped>>("PoseInBlackboard");

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
