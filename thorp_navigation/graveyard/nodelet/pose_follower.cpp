/*
 * Author: Jorge Santos
 * Based on https://github.com/turtlebot/turtlebot_apps/blob/indigo/turtlebot_follower/src/follower.cpp
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <turtlebot_msgs/SetFollowState.h>
#include <dynamic_reconfigure/server.h>

#include <thorp_toolkit/tf.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_navigation/FollowerConfig.h"


namespace thorp_navigation
{

/**
 * The pose follower nodelet. Subscribes to a pose and publishes command vel messages to reach it.
 */
class PoseFollower : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the follower.
   */
  PoseFollower() : d_target_(1.0), v_scale_(1.0), w_scale_(5.0)
  {
  }

  virtual ~PoseFollower()
  {
    delete config_srv_;
  }

  /*!
   * @brief init method; read non-dynamic parameters and set up ROS interface.
   */
  virtual void onInit()
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("robot_frame", robot_frame_, std::string("base_footprint"));
    private_nh.param("no_pose_timeout", no_pose_timeout_, 0.5);

    twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pose_sub_ = nh.subscribe("target_pose", 1, &PoseFollower::poseCallback, this);

    switch_srv_ = private_nh.advertiseService("change_state", &PoseFollower::changeModeSrvCb, this);

    config_srv_ = new dynamic_reconfigure::Server<thorp_navigation::FollowerConfig>(private_nh);
    dynamic_reconfigure::Server<thorp_navigation::FollowerConfig>::CallbackType f =
      boost::bind(&PoseFollower::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);

    // Create (stopped by now) a one-shot timer to stop the robot if no more poses are received within no_pose_timeout
    no_pose_timer_ =
      private_nh.createTimer(ros::Duration(no_pose_timeout_), &PoseFollower::timerCallback, this, true, false);
  }

private:
  double d_target_; /**< The closest distance to the target we can reach */
  double v_scale_;  /**< The scaling factor for translational robot speed */
  double w_scale_;  /**< The scaling factor for rotational robot speed */
  bool enabled_;    /**< Enable/disable following; just prevents motor commands */
  std::string robot_frame_; /**< Reference frame, used to calculate distance and heading errors */

  ros::Subscriber pose_sub_;
  ros::Publisher twist_pub_;

  ros::Timer no_pose_timer_;  /**< No pose messages received timer */
  double no_pose_timeout_;    /**< No pose messages received timeout */

  // Service for start/stop following
  ros::ServiceServer switch_srv_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<thorp_navigation::FollowerConfig> *config_srv_;

  void reconfigure(thorp_navigation::FollowerConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
    v_scale_ = config.v_scale;
    w_scale_ = config.w_scale;
    d_target_ = config.distance;
    ROS_DEBUG("Reconfigured: %d %g %g %g", enabled_, v_scale_, w_scale_, d_target_);
  }

  /*!
   * @brief Callback for target pose.
   * Publishes cmd_vel messages to reach the pose up to a preset distance.
   * @param msg The target pose message.
   */
  void poseCallback(const geometry_msgs::PoseStamped &msg)
  {
    if (!enabled_)
      return;

    // transform target pose into robot reference frame
    geometry_msgs::PoseStamped target_pose;
    ttk::transformPose(msg.header.frame_id, robot_frame_, msg, target_pose);

    // calculate distance and heading to the target pose
    double x = target_pose.pose.position.x;
    double y = target_pose.pose.position.y;
    double z = target_pose.pose.position.z;
    double distance = ttk::distance2D(target_pose);
    double heading = ttk::heading(target_pose);

    geometry_msgs::Twist cmd;
    if (std::abs(heading) > M_PI/2.0)
    {
      cmd.angular.z = heading * w_scale_;
      ROS_INFO_THROTTLE(1, "Rotating at %.2frad/s to target pose %.2f %.2f; distance: %.2f, heading: %.2f",
                        cmd.angular.z, x, y, distance, heading);
    }
    else
    {
      cmd.linear.x = (distance - d_target_) * v_scale_;
      cmd.angular.z = heading * w_scale_;
      ROS_INFO_THROTTLE(1, "Following at %.2fm/s, %.2frad/s target pose %.2f %.2f; distance: %.2f, heading: %.2f",
                        cmd.linear.x, cmd.angular.z, x, y, distance, heading);
    }
    twist_pub_.publish(cmd);

    // Reset timeout to stop the robot if no more poses are received within a short time
    no_pose_timer_.stop();
    no_pose_timer_.start();
  }

  void timerCallback(const ros::TimerEvent& event)
  {
    ROS_INFO("No pose msgs received for %gs: following stopped", no_pose_timeout_);
    twist_pub_.publish(geometry_msgs::Twist());
  }

  bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request &request,
                       turtlebot_msgs::SetFollowState::Response &response)
  {
    if (enabled_ && request.state==request.STOPPED)
    {
      ROS_INFO("Change mode service request: following stopped");
      twist_pub_.publish(geometry_msgs::Twist());
      enabled_ = false;
    }
    else if (!enabled_ && request.state==request.FOLLOW)
    {
      ROS_INFO("Change mode service request: following (re)started");
      enabled_ = true;
    }

    response.result = response.OK;
    return true;
  }

};

PLUGINLIB_EXPORT_CLASS(thorp_navigation::PoseFollower, nodelet::Nodelet)

}  // thorp_navigation namespace
