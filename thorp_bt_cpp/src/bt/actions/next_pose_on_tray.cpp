#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/visualization.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Calculate the next pose where to put an object on the tray.
 */
class NextPoseOnTray : public BT::SyncActionNode
{
public:
  NextPoseOnTray(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), tray_full_(false), next_x_(0), next_y_(0)
  {
    ros::NodeHandle pnh("~");
    tray_link_ = pnh.param("tray_link", std::string("tray_link"));
    tray_slot_ = pnh.param("tray_slot", 0.035);
    const double tray_side_x = pnh.param("tray_side_x", 0.14);
    const double tray_side_y = pnh.param("tray_side_y", 0.14);
    slots_x_ = std::round(tray_side_x / tray_slot_ + 0.1);
    slots_y_ = std::round(tray_side_y / tray_slot_ + 0.1);
    offset_x_ = (slots_x_ % 2 == 0) ? tray_slot_ / 2.0 : 0.0;
    offset_y_ = (slots_y_ % 2 == 0) ? tray_slot_ / 2.0 : 0.0;
    offset_z_ = pnh.param("placing_height_on_tray", 0.03);

    ROS_DEBUG_NAMED(name, "Tray dimensions: %g x %g m. %d x %d slots of %g x %g m each",
                    tray_side_x, tray_side_y, slots_x_, slots_y_, tray_slot_, tray_slot_);

    visualizePlacePoses();
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("tray_link"),  //
             BT::OutputPort<geometry_msgs::PoseStamped>("pose_on_tray") };
  }

private:
  std::string tray_link_;
  double tray_slot_;
  bool tray_full_;
  int slots_x_;
  int slots_y_;
  double offset_x_;
  double offset_y_;
  double offset_z_;
  int next_x_;
  int next_y_;

  BT::NodeStatus tick() override
  {
    if (tray_full_)
    {
      ROS_WARN_NAMED(name(), "Tray is full");
      return BT::NodeStatus::FAILURE;
    }

    auto pose_on_tray = nextPose(offset_z_);
    setOutput("pose_on_tray", pose_on_tray);
    ROS_INFO_NAMED(name(), "Next pose on tray at %s", ttk::toCStr3D(pose_on_tray));
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::PoseStamped nextPose(double z)
  {
    ROS_ASSERT_MSG(!tray_full_, "Calling nextPose when the tray is already full");

    // Get next empty location coordinates
    double x = (next_x_ - slots_x_ / 2) * tray_slot_ + offset_x_;
    double y = (next_y_ - slots_y_ / 2) * tray_slot_ + offset_y_;
    next_x_ = (next_x_ + 1) % slots_x_;
    if (next_x_ == 0)
    {
      next_y_ = (next_y_ + 1) % slots_y_;
      if (next_x_ == next_y_)
      {
        tray_full_ = true;
      }
    }

    geometry_msgs::PoseStamped pose = ttk::createPose(x, y, z, 0, 0, 0, tray_link_);
    return pose;
  }

  void visualizePlacePoses()
  {
    geometry_msgs::PoseStamped ref_pose = ttk::createPose(0, 0, 0, tray_link_);
    std::vector<geometry_msgs::Point> points;
    for (int i = 0; i < slots_x_ * slots_y_; ++i)
    {
      points.push_back(nextPose(0.01).pose.position);
    }
    std::vector<std_msgs::ColorRGBA> colors(points.size(), ttk::namedColor("red"));

    ttk::Visualization viz;
    viz.addMarkers({ ttk::Visualization::createPointList(ref_pose, points, colors, 0.01) });
    viz.publishMarkers();

    tray_full_ = false;  // calling nextPose for all slots will make the tray "full"!
  }

  BT_REGISTER_NODE(NextPoseOnTray);
};
}  // namespace thorp::bt::actions
