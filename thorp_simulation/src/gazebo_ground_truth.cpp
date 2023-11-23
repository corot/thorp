/**
 * Use Gazebo ground truth for perfect localization
 */

#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/LinkStates.h>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

ros::Duration period;

void linkStatesCB(const gazebo_msgs::LinkStates& msg)
{
  static ros::Time last_pub_time;
  if (ros::Time::now() - last_pub_time < period)
    return;

  // get map -> base from gazebo
  auto it = std::find(msg.name.begin(), msg.name.end(), "thorp::base_footprint");
  if (it == msg.name.end())
  {
    ROS_INFO_THROTTLE(2.0, "Waiting for thorp model to provide ground truth localization");
    return;
  }
  tf2::Transform map_to_bfp_tf2;
  size_t index = it - msg.name.begin();
  tf2::convert(msg.pose[index], map_to_bfp_tf2);

  // subtract (multiply by the inverse) odom -> base_footprint tf
  tf2::Transform bfp_to_odom_tf2;
  geometry_msgs::TransformStamped bfp_to_odom_tf;
  if (!ttk::TF2::instance().lookupTransform("base_footprint", "odom", bfp_to_odom_tf))
    return;
  tf2::convert(bfp_to_odom_tf.transform, bfp_to_odom_tf2);

  tf2::Transform map_to_odom_tf2 = map_to_bfp_tf2 * bfp_to_odom_tf2;
  geometry_msgs::TransformStamped map_to_odom_tf;
  map_to_odom_tf.header.stamp = ros::Time::now();
  map_to_odom_tf.header.frame_id = "map";
  map_to_odom_tf.child_frame_id = "odom";
  tf2::convert(map_to_odom_tf2, map_to_odom_tf.transform);
  ttk::TF2::instance().sendTransform(map_to_odom_tf);
  last_pub_time = map_to_odom_tf.header.stamp;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_ground_truth");

  ros::NodeHandle nh, pnh("~");
  period.fromSec(1.0 / pnh.param("frequency", 20.0));
  ros::Subscriber sub = nh.subscribe("gazebo/link_states", 1, linkStatesCB);
  ros::spin();
  return 0;
}

