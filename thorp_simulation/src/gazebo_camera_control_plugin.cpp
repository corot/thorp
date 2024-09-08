#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/UserCamera.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

/**
 * Subscribe to RViz camera pose (provided by the AnimatedViewController plugin) and apply
 * the incoming poses to the Gazebo camera, so we mimic the view on both applications.
 */
class CameraPosePlugin : public gazebo::VisualPlugin
{
public:
  CameraPosePlugin() : VisualPlugin()
  {
  }

  void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override
  {
    // Connect pre-render events to onUpdate method, so we can update the camera pose
    gazebo::rendering::ScenePtr scene = visual->GetScene();
    camera_ = scene->GetUserCamera(0);
    if (!camera_)
    {
      // This happens on the 2nd instance this plugin gets loaded; I guess that's headless gazebo server
      return;
    }

    update_connection_ = gazebo::event::Events::ConnectPreRender([this] { onUpdate(); });

    // Subscribe to the ROS topic to obtain RViz camera pose
    sub_ = ros::NodeHandle().subscribe("rviz/current_camera_pose", 1, &CameraPosePlugin::poseCallback, this);

    ROS_INFO("CameraPosePlugin loaded and waiting for camera pose messages...");
  }

  void onUpdate()
  {
    geometry_msgs::PoseStamped pose_mrf;
    {
      std::scoped_lock l(rviz_pose_mutex_);
      if (!rviz_pose_ || !ttk::TF2::instance().transformPose("map", *rviz_pose_, pose_mrf))
      {
        return;
      }
    }

    ignition::math::Pose3d camera_pose;
    double roll, pitch, yaw;
    tf::Quaternion q;
    quaternionMsgToTF(pose_mrf.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    camera_pose.Set(pose_mrf.pose.position.x, pose_mrf.pose.position.y, pose_mrf.pose.position.z, roll, pitch, yaw);

    //    ROS_DEBUG("Processed camera pose: %.2f %.2f %.2f   %.2f %.2f %.2f", pose_mrf.pose.position.x,
    //              pose_mrf.pose.position.y, pose_mrf.pose.position.z, roll, pitch, yaw);

    camera_->SetWorldPose(camera_pose);
  }

  void poseCallback(const geometry_msgs::PoseStamped& msg)
  {
    std::scoped_lock l(rviz_pose_mutex_);
    rviz_pose_ = msg;
    rviz_pose_->header.stamp.fromSec(0.0);
    // RViz only publishes this pose when it changes, so in order to follow moving frames,
    // we need to reset time so we always transform to the latest available map -> pose tf
  }

private:
  ros::Subscriber sub_;
  std::mutex rviz_pose_mutex_;
  std::optional<geometry_msgs::PoseStamped> rviz_pose_;
  gazebo::rendering::UserCameraPtr camera_;
  gazebo::event::ConnectionPtr update_connection_;
};

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(CameraPosePlugin)
