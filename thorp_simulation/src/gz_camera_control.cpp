/**
 * Subscribe to RViz camera pose (provided by the AnimatedViewController plugin) and republish
 * to Gazebo topic user_camera/joy_pose, so we mimic the view on both applications.
 */

#include <tf/tf.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>


gazebo::transport::PublisherPtr gz_cam_pub;

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
  ignition::math::Pose3d pose;
  double roll, pitch, yaw;
  tf::Quaternion q;
  quaternionMsgToTF(msg.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  pose.Set(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw);
  gazebo::msgs::Pose gz_pose;
  gazebo::msgs::Set(&gz_pose, pose);
  ROS_DEBUG("Camera pose: %.2f\t%.2f\t%.2f   \t%.2f\t%.2f\t%.2f",
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw);
  gz_cam_pub->Publish(gz_pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gz_camera_control");

  // Gazebo camera interface
  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr gz_node(new gazebo::transport::Node());
  gz_node->Init();
  gz_cam_pub = gz_node->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");

  // RViz camera interface
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("rviz/camera_pose", 1, poseCallback);

  ros::spin();
  return 0;
}
