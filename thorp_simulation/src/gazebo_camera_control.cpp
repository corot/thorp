/**
 * Subscribe to RViz camera pose (provided by the AnimatedViewController plugin) and republish
 * to Gazebo topic user_camera/joy_pose, so we mimic the view on both applications.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>


gazebo::transport::PublisherPtr gz_cam_pub;

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
  geometry_msgs::PoseStamped pose_mrf = msg;
  ttk::TF2::instance().transformPose("map", msg, pose_mrf);

  ignition::math::Pose3d camera_pose;
  double roll, pitch, yaw;
  tf::Quaternion q;
  quaternionMsgToTF(pose_mrf.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  camera_pose.Set(pose_mrf.pose.position.x, pose_mrf.pose.position.y, pose_mrf.pose.position.z, roll, pitch, yaw);
  gazebo::msgs::Pose gz_pose;
  gazebo::msgs::Set(&gz_pose, camera_pose);
  ROS_DEBUG("Camera pose: %.2f\t%.2f\t%.2f   \t%.2f\t%.2f\t%.2f",
            pose_mrf.pose.position.x, pose_mrf.pose.position.y, pose_mrf.pose.position.z, roll, pitch, yaw);
  gz_cam_pub->Publish(gz_pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_camera_control");

  // Gazebo camera interface
  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr gz_node(new gazebo::transport::Node());
  if (!gz_node->TryInit(gazebo::common::Time(60)))
  {
    ROS_ERROR("Gazebo global namespace was not found after 1 minute");
    return -1;
  }
  gz_cam_pub = gz_node->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");

  // RViz camera interface
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("rviz/camera_pose", 1, poseCallback);

  ros::spin();
  gz_node->Fini();
  return 0;
}
