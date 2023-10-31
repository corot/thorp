#include "thorp_bt_cpp/bt_nodes.hpp"

// bt actions
#include "thorp_bt_cpp/bt/actions/clear_costmaps.hpp"
#include "thorp_bt_cpp/bt/actions/get_path_action.hpp"
#include "thorp_bt_cpp/bt/actions/exe_path_action.hpp"
#include "thorp_bt_cpp/bt/actions/recovery_action.hpp"
#include "thorp_bt_cpp/bt/actions/move_base_action.hpp"
#include "thorp_bt_cpp/bt/actions/get_robot_pose.hpp"
#include "thorp_bt_cpp/bt/actions/select_controller.hpp"
#include "thorp_bt_cpp/bt/actions/set_blackboard.hpp"
#include "thorp_bt_cpp/bt/actions/segment_rooms.hpp"
#include "thorp_bt_cpp/bt/actions/plan_room_sequence.hpp"
#include "thorp_bt_cpp/bt/actions/plan_room_exploration.hpp"

// bt conditions
#include "thorp_bt_cpp/bt/conditions/is_bool_true.hpp"

// bt controls
#include "thorp_bt_cpp/bt/controls/super_reactive_sequence.hpp"

// bt decorators
#include "thorp_bt_cpp/bt/decorators/store_result.hpp"

// bt tools
#include <behaviortree_cpp_v3/xml_parsing.h>

// others
#include "thorp_bt_cpp/bt/utils.hpp"

#include <iostream>
#include <fstream>
#include <string>

namespace thorp::bt
{

void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh, const std::string& nodes_file_path)
{
  // actions
  registerNode<bt::actions::ClearCostmaps>(factory, nh, "ClearCostmaps");
  registerNode<bt::actions::GetRobotPose>(factory, nh, "GetRobotPose");
  //    registerNode<bt::actions::SelectController>("SelectController");
  //    registerNode<bt::actions::SetBlackboard<bool>>("SetBool");
  //    registerNode<bt::actions::SetBlackboard<double>>("SetDouble");
  //    registerNode<bt::actions::SetBlackboard<unsigned int>>("SetUnsignedInt");
  BT::RegisterSimpleActionClient<bt::actions::GetPathAction>(factory, "GetPathAction");
  BT::RegisterSimpleActionClient<bt::actions::ExePathAction>(factory, "ExePathAction");
  BT::RegisterSimpleActionClient<bt::actions::RecoveryAction>(factory, "RecoveryAction");
  BT::RegisterSimpleActionClient<bt::actions::MoveBaseAction>(factory, "MoveBaseAction");
  BT::RegisterSimpleActionClient<bt::actions::SegmentRooms>(factory, "SegmentRooms");
  BT::RegisterSimpleActionClient<bt::actions::PlanRoomSequence>(factory, "PlanRoomSequence");
  BT::RegisterSimpleActionClient<bt::actions::PlanRoomExploration>(factory, "PlanRoomExploration");

  // conditions
  //    registerNode<bt::conditions::IsBoolTrue>("IsBoolTrue");

  // controls
  registerNode<bt::controls::SuperReactiveSequence>(factory, nh, "SuperReactiveSequence");

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
