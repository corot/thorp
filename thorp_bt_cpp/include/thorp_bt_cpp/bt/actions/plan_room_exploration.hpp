#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "thorp_bt_cpp/bt/utils.hpp"
#include "thorp_bt_cpp/bt/actions/action_client_node.hpp"

#include <geometry_msgs/PolygonStamped.h>
#include <ipa_building_msgs/RoomExplorationAction.h>

#include <thorp_toolkit/common.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp_toolkit;

namespace thorp::bt::actions
{
class PlanRoomExploration : public BT::SimpleActionClientNode<ipa_building_msgs::RoomExplorationAction>
{
public:
  PlanRoomExploration(const std::string& name, const BT::NodeConfiguration& config)
    : SimpleActionClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::PoseStamped>("robot_pose"),
             BT::InputPort<float>("robot_radius"),
             BT::InputPort<sensor_msgs::Image>("map_image"),
             BT::InputPort<geometry_msgs::Pose>("map_origin"),
             BT::InputPort<float>("map_resolution"),
             BT::InputPort<sensor_msgs::Image>("segmented_map"),
             BT::InputPort<std::vector<ipa_building_msgs::RoomInformation>>("room_information_in_meter"),
             BT::InputPort<uint32_t>("room_number"),
             BT::OutputPort<geometry_msgs::PoseStamped>("start_pose"),
             BT::OutputPort<std::vector<geometry_msgs::PoseStamped>>("coverage_path") };
  }

  bool createGoal(GoalType& goal)
  {
    // We need an image containing only the room to explore, so we copy the original map and
    // set to black all pixels not belonging to the given room number in the segmented map
    uint32_t room_number = *getInput<uint32_t>("room_number");
    sensor_msgs::Image segmented_map = *getInput<sensor_msgs::Image>("segmented_map");
    sensor_msgs::Image one_room_map = *getInput<sensor_msgs::Image>("map_image");
    std::set<int> r;
    for (int i = 0, j = 0; i < segmented_map.data.size();)
    {
      int room = int((uint8_t)(segmented_map.data[i++]) |
                     (uint8_t)(segmented_map.data[i++]) << 8 |
                     (uint8_t)(segmented_map.data[i++]) << 16 |
                     (uint8_t)(segmented_map.data[i++]) << 24);
      r.insert(room);
      if (room != room_number)
        one_room_map.data[j] = 0;
      ++j;
    }
    std::for_each(r.begin(), r.end(), [](int ri) { printf("%d  ", ri); });
    printf("\n\n");

    ROS_ERROR_STREAM(room_number);
    geometry_msgs::Pose robot_pose = getInput<geometry_msgs::PoseStamped>("robot_pose")->pose;
    geometry_msgs::TransformStamped bfp_cam_tf;
    ttk::TF2::instance().lookupTransform("kinect_rgb_frame", "base_footprint", bfp_cam_tf);

    goal.robot_radius = *getInput<float>("robot_radius");
    goal.map_origin = *getInput<geometry_msgs::Pose>("map_origin");
    goal.map_resolution = *getInput<float>("map_resolution");
    goal.input_map = one_room_map;
    goal.planning_mode = 2;  // plan a path for coverage with the robot's field of view
    goal.starting_position = ttk::toPose2D(robot_pose);
    goal.field_of_view = loadFOVParam();
    goal.field_of_view_origin.x = bfp_cam_tf.transform.translation.x;
    goal.field_of_view_origin.y = bfp_cam_tf.transform.translation.y;
    goal.field_of_view_origin.z = bfp_cam_tf.transform.translation.z;
/*
 * TODO return fov as a PolygonStamped to publish while exploring
    sensor_msgs::Image img_msg;
    // Populate img_msg with the input map data

    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/exploration/img", 1, true);
    img_pub.publish(img_msg);

    ros::Timer fov_pub_timer = nh.createTimer(ros::Duration(0.1),
                                              [&](const ros::TimerEvent& event)
                                              {
                                                ros::Publisher fov_pub = nh.advertise<geometry_msgs::PolygonStamped>(
                                                    "/exploration/fov", 1, true);
                                                fov_pub.publish(fov);
                                              });*/
    return true;
  }

  BT::NodeStatus onSuccess(const ResultConstPtr& res)
  {
    // Use the coverage path provided as a list of stamped poses
    setOutput("coverage_path", res->coverage_path_pose_stamped);

    return BT::NodeStatus::SUCCESS;
  }

  std::vector<geometry_msgs::Point32> loadFOVParam()
  {
    std::vector<geometry_msgs::Point32> fov_points;
    XmlRpc::XmlRpcValue fov_param;
    if (!ros::NodeHandle().getParam("exploration/room_exploration/field_of_view_points", fov_param))
      throw ros::InvalidParameterException("Exploration FOV parameter not found");

    if (fov_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
      throw ros::InvalidParameterException("Exploration FOV parameter is not a list");

    fov_points.resize(fov_param.size());
    for (int i = 0; i < fov_param.size(); ++i)
    {
      if (fov_param[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw ros::InvalidParameterException("Exploration FOV parameter is not a nested list");

      if (fov_param[i].size() != 3)
        throw ros::InvalidParameterException("Exploration FOV parameter nested list is not a triplet");

      if (fov_param[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
          fov_param[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
          fov_param[i][2].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        throw ros::InvalidParameterException("Exploration FOV parameter nested list is not a triplet of floats");

      fov_points[i].x = static_cast<double>(fov_param[i][0]);
      fov_points[i].y = static_cast<double>(fov_param[i][1]);
      fov_points[i].z = static_cast<double>(fov_param[i][2]);
    }
    return fov_points;
  }
};
}  // namespace thorp::bt::actions
