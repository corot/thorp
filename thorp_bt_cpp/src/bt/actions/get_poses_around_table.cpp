#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <mbf_msgs/CheckPose.h>
#include <geometry_msgs/PoseArray.h>
#include <rail_manipulation_msgs/SegmentedObject.h>

#include <thorp_toolkit/parameters.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Calculate pickup locations around a table at a given distance, discarding those blocked on global costmap.
 */
class GetPosesAroundTable : public BT::SyncActionNode
{
public:
  GetPosesAroundTable(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
    ttk::getParam("max_arm_reach", max_arm_reach_);
    ttk::getParam("table_min_pickup_side", min_pickup_side_);

    ros::NodeHandle pnh("~");
    valid_poses_pub_ = pnh.advertise<geometry_msgs::PoseArray>("valid_poses_around_table", 1);
    blocked_poses_pub_ = pnh.advertise<geometry_msgs::PoseArray>("blocked_poses_around_table", 1);

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
             BT::InputPort<bool>("split_long_sides", false, "Create two poses on sides longer than max_arm_reach x 2"),
             BT::OutputPort<std::vector<geometry_msgs::PoseStamped>>("table_side_poses") };
  }

private:
  double max_arm_reach_;
  double min_pickup_side_;
  ros::Publisher valid_poses_pub_;
  ros::Publisher blocked_poses_pub_;
  ros::ServiceClient check_pose_srv_;

  BT::NodeStatus tick() override
  {
    const auto table = *getInput<rail_manipulation_msgs::SegmentedObject>("table");
    const auto table_pose = *getInput<geometry_msgs::PoseStamped>("table_pose");
    const auto split_long = *getInput<bool>("split_long_sides");
    const auto distance = *getInput<double>("distance");

    // Create 0, 1 or 2 poses for each of the four sides around the table
    const double p_x = distance + table.depth / 2.0;
    const double n_x = -p_x;
    const double p_y = distance + table.width / 2.0;
    const double n_y = -p_y;

    auto make_poses = [&](double x, double y, double t, double l) -> std::vector<geometry_msgs::PoseStamped>
    {
      // none, too narrow to pickup
      if (l < min_pickup_side_)
        return {};
      // one at the center of the side
      if (l < max_arm_reach_ * 2.0 || !split_long)
        return { ttk::createPose(x, y, t, "table_frame") };
      // split the side into two poses
      return { ttk::createPose(x == 0.0 ? -l / 4.0 : x, y == 0.0 ? -l / 4.0 : y, t, "table_frame"),
               ttk::createPose(x == 0.0 ? +l / 4.0 : x, y == 0.0 ? +l / 4.0 : y, t, "table_frame") };
    };

    auto tmp_all = { std::move(make_poses(p_x, 0.0, -M_PI, table.width)),
                     std::move(make_poses(n_x, 0.0, 0.0, table.width)),
                     std::move(make_poses(0.0, p_y, -M_PI / 2.0, table.depth)),
                     std::move(make_poses(0.0, n_y, M_PI / 2.0, table.depth)) };
    std::vector<geometry_msgs::PoseStamped> poses;
    for (const auto& tmp : tmp_all)
    {
      poses.insert(poses.end(), tmp.begin(), tmp.end());
    }

    auto table_tf = ttk::toTransform(table_pose);
    table_tf.transform.translation.z = 0.0;  // as we want poses at floor level

    // Transform table side poses to global frame
    std::for_each(poses.begin(), poses.end(),
                  [&](geometry_msgs::PoseStamped& pose) { tf2::doTransform(pose, pose, table_tf); });

    // Remove blocked poses
    std::vector<geometry_msgs::PoseStamped> removed_poses;
    std::copy_if(poses.begin(), poses.end(), std::back_inserter(removed_poses),
                 [this](const geometry_msgs::PoseStamped& pose) { return isBlocked(pose); });
    ROS_ERROR_COND_NAMED(!removed_poses.empty(), name(), "%lu poses out of %lu discarded as blocked",
                         removed_poses.size(), poses.size());

    poses.erase(std::remove_if(poses.begin(), poses.end(), [&](const geometry_msgs::PoseStamped& pose)
                       { return std::find(removed_poses.begin(), removed_poses.end(), pose) != removed_poses.end(); }),
                poses.end());

    valid_poses_pub_.publish(ttk::toPoseArray(poses, 0.01, table_tf.header.frame_id));
    blocked_poses_pub_.publish(ttk::toPoseArray(removed_poses, 0.01, table_tf.header.frame_id));

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
    srv.request.safety_dist = -0.15;  // pickup poses can be within the table, and so in collision
    if (!check_pose_srv_.call(srv))
    {
      ROS_WARN_NAMED(name(), "MBF check pose service failed; assume pose is not blocked");
      return false;
    }
    if (srv.response.state >= mbf_msgs::CheckPoseResponse::LETHAL)
    {
      ROS_ERROR_NAMED(name(), "Blocked pose %s: %d; cost: %u", ttk::toCStr2D(pose), (int)srv.response.state,
                      srv.response.cost);
      return true;
    }
    ROS_DEBUG_NAMED(name(), "Reachable pose %s: %d; cost: %u", ttk::toCStr2D(pose), (int)srv.response.state,
                    srv.response.cost);
    return false;
  }

  BT_REGISTER_NODE(GetPosesAroundTable);
};
}  // namespace thorp::bt::actions