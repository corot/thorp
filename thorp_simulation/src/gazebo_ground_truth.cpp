/**
 * Use Gazebo ground truth for perfect localization: publish the map -> odom transform that makes map -> base_footprint
 * match the robot pose in Gazebo world, provided by Gazebo's odometry publisher on ground_truth/odom
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <thorp_toolkit/common.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gazebo_ground_truth");
  ttk::init(node);

  rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / node->declare_parameter("frequency", 20.0));
  rclcpp::Time last_pub_time(0, 0, node->get_clock()->get_clock_type());

  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "ground_truth/odom", 1, [&](const nav_msgs::msg::Odometry& msg) {
        if (node->now() - last_pub_time < period)
          return;

        // get map -> base from gazebo
        tf2::Transform map_to_bfp_tf2;
        tf2::fromMsg(msg.pose.pose, map_to_bfp_tf2);

        // subtract (multiply by the inverse) odom -> base_footprint tf
        tf2::Transform bfp_to_odom_tf2;
        geometry_msgs::msg::TransformStamped bfp_to_odom_tf;
        if (!ttk::TF2::instance().lookupTransform("base_footprint", "odom", bfp_to_odom_tf))
          return;
        tf2::fromMsg(bfp_to_odom_tf.transform, bfp_to_odom_tf2);

        tf2::Transform map_to_odom_tf2 = map_to_bfp_tf2 * bfp_to_odom_tf2;
        geometry_msgs::msg::TransformStamped map_to_odom_tf;
        map_to_odom_tf.header.stamp = node->now();
        map_to_odom_tf.header.frame_id = "map";
        map_to_odom_tf.child_frame_id = "odom";
        map_to_odom_tf.transform = tf2::toMsg(map_to_odom_tf2);
        ttk::TF2::instance().sendTransform(map_to_odom_tf);
        last_pub_time = map_to_odom_tf.header.stamp;
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
