#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <mbf_msgs/CheckPose.h>
#include <rail_manipulation_msgs/SegmentedObject.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Calculate the four locations around a table at a given distance, discarding those blocked on global costmap.
 */
class GetPosesAroundTable : public BT::SyncActionNode
{
public:
  GetPosesAroundTable(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
    if (ros::service::waitForService("move_base_flex/check_pose_cost", ros::Duration(30)))
    {
      check_pose_srv_ = ros::NodeHandle().serviceClient<mbf_msgs::CheckPose>("move_base_flex/check_pose_cost", true);
    }
    else
    {
      ROS_WARN_NAMED(name, "Unable to connect to MBF's check_pose_cost service; we won't filter blocked poses");
    }
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<rail_manipulation_msgs::SegmentedObject>("table"),  //
             BT::InputPort<geometry_msgs::PoseStamped>("table_pose"),          //
             BT::InputPort<double>("distance"),                                //
             BT::OutputPort<std::vector<geometry_msgs::PoseStamped>>("table_side_poses") };
  }

private:
  ros::ServiceClient check_pose_srv_;

  BT::NodeStatus tick() override
  {
    const auto table = *getInput<rail_manipulation_msgs::SegmentedObject>("table");
    const auto table_pose = *getInput<geometry_msgs::PoseStamped>("table_pose");
    const auto distance = *getInput<double>("distance");

    // Create poses for the four sides around the table
    const double p_x = distance + table.depth / 2.0;
    const double n_x = -p_x;
    const double p_y = distance + table.width / 2.0;
    const double n_y = -p_y;

    std::vector<geometry_msgs::PoseStamped> poses = { ttk::createPose(p_x, 0.0, -M_PI, "table_frame"),
                                                      ttk::createPose(n_x, 0.0, 0.0, "table_frame"),
                                                      ttk::createPose(0.0, p_y, -M_PI / 2.0, "table_frame"),
                                                      ttk::createPose(0.0, n_y, M_PI / 2.0, "table_frame") };
    auto table_tf = ttk::toTransform(table_pose);
    table_tf.transform.translation.z = 0.0;  // as we want poses at floor level

    // Transform table side poses to global frame
    std::for_each(poses.begin(), poses.end(),
                  [&](geometry_msgs::PoseStamped& pose) { tf2::doTransform(pose, pose, table_tf); });

    // Remove blocked poses
    poses.erase(std::remove_if(poses.begin(), poses.end(),
                               [this](const geometry_msgs::PoseStamped& pose) { return isBlocked(pose); }),
                poses.end());
    ROS_ERROR_STREAM_COND_NAMED(4 - poses.size(), name(), 4 - poses.size() << " poses discarded as blocked");

    setOutput("table_side_poses", poses);

    return BT::NodeStatus::SUCCESS;
  }

  bool isBlocked(const geometry_msgs::PoseStamped& pose)
  {
    if (!check_pose_srv_)
    {
      return false;  // no check service available; we already warned on the constructor
    }

    // Evaluate cost of the footprint at the given pose
    mbf_msgs::CheckPose srv;
    srv.request.pose = pose;
    srv.request.costmap = mbf_msgs::CheckPoseRequest::GLOBAL_COSTMAP;
    srv.request.safety_dist = -0.1;
    if (!check_pose_srv_.call(srv))
    {
      ROS_WARN_NAMED(name(), "MBF check pose service failed; assume pose is not blocked");
      return false;
    }
    if (srv.response.state >= mbf_msgs::CheckPoseResponse::LETHAL)
    {
      // Highlight the problematic pose to allow easy debugging
      ROS_ERROR_NAMED(name(), "Blocked pose %s: %d; cost: %ud", ttk::toCStr2D(pose), (int)srv.response.state,
                      srv.response.cost);
      return true;
    }
    ROS_INFO_NAMED(name(), "fine pose %s: %d; cost: %ud", ttk::toCStr2D(pose), (int)srv.response.state,
                    srv.response.cost);
    return false;
  }

  BT_REGISTER_NODE(GetPosesAroundTable);
};
}  // namespace thorp::bt::actions