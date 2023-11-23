/*
 * Author: Jorge Santos
 * Based on https://github.com/turtlebot/turtlebot_apps/blob/indigo/turtlebot_follower/src/follower.cpp
 */
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/FollowPoseAction.h>
#include <dynamic_reconfigure/server.h>

#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

#include "thorp_navigation/FollowerConfig.h"


namespace thorp::navigation
{

/**
 * The pose follower node. Subscribes to a pose and approaches it up to a given distance.
 * At that distance, it keeps heading toward the target pose.
 */
class PoseFollower
{
public:
  /*!
   * @brief The constructor for the follower.
   */
  PoseFollower()
      : d_target_(1.0), v_scale_(1.0), w_scale_(1.0), frequency_(10.0), robot_frame_("base_footprint"),
        pnh_("~"), follow_as_(pnh_, "follow", boost::bind(&PoseFollower::executeCB, this, _1), false)
  {
  }

  virtual ~PoseFollower()
  {
    delete config_srv_;
  }

  /*!
   * @brief init method; read non-dynamic parameters and set up ROS interface.
   */
  virtual void init()
  {
    ros::NodeHandle nh;

    double no_pose_timeout;
    pnh_.param("frequency", frequency_, frequency_);
    pnh_.param("robot_frame", robot_frame_, robot_frame_);
    pnh_.param("no_pose_timeout", no_pose_timeout, 1.0);
    no_pose_timeout_.fromSec(no_pose_timeout);

    twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pose_sub_ = nh.subscribe("target_pose", 1, &PoseFollower::poseCallback, this);

    config_srv_ = new dynamic_reconfigure::Server<thorp_navigation::FollowerConfig>(pnh_);
    dynamic_reconfigure::Server<thorp_navigation::FollowerConfig>::CallbackType f =
      boost::bind(&PoseFollower::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);

    follow_as_.start();
  }

private:
  double d_target_; /**< The closest distance to the target we can reach */
  double v_scale_;  /**< The scaling factor for translational robot speed */
  double w_scale_;  /**< The scaling factor for rotational robot speed */
  double frequency_;
  bool enabled_;    /**< Enable/disable following; just prevents motor commands */
  std::string robot_frame_; /**< Reference frame, used to calculate distance and heading errors */

  ros::NodeHandle pnh_;

  ros::Subscriber pose_sub_;
  ros::Publisher twist_pub_;

  ros::Time last_pose_time_;         /**< Last received pose messages time */
  ros::Duration no_pose_timeout_;    /**< No pose messages received timeout */
  geometry_msgs::PoseStamped target_pose_;
  std::mutex target_pose_mutex_;

  actionlib::SimpleActionServer<thorp_msgs::FollowPoseAction> follow_as_;
  thorp_msgs::FollowPoseFeedback feedback_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<thorp_navigation::FollowerConfig>* config_srv_;

  void reconfigure(thorp_navigation::FollowerConfig& config, uint32_t level)
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
    std::lock_guard<std::mutex> lg(target_pose_mutex_);
    last_pose_time_ = ros::Time::now();  // so we don't relay on pose's stamp
    target_pose_ = msg;
  }

  void executeCB(const thorp_msgs::FollowPoseGoal::ConstPtr &goal)
  {
    thorp_msgs::FollowPoseResult result;

    double follow_distance = goal->distance > 0.0 ? goal->distance : d_target_;
    ROS_INFO("Following target pose at %g m", follow_distance);
    ros::Rate rate(frequency_);
    ros::Time start_time = ros::Time::now();
    last_pose_time_ = ros::Time();  // ensure we start with a newly received pose

    while (ros::ok())
    {
      if (follow_as_.isPreemptRequested())
      {
        result.outcome = thorp_msgs::FollowPoseResult::FOLLOW_CANCELED;
        follow_as_.setPreempted(result);
        ROS_INFO("Following preempted");
        break;
      }

      if (!goal->time_limit.isZero() && ros::Time::now() - start_time > goal->time_limit)
      {
        result.outcome = thorp_msgs::FollowPoseResult::RUN_OUT_OF_TIME;
        follow_as_.setSucceeded(result);
        ROS_INFO("Following time limit of %g seconds reached", goal->time_limit.toSec());
        break;
      }

      geometry_msgs::PoseStamped target_pose_in_robot_frame;
      {
        std::lock_guard<std::mutex> lg(target_pose_mutex_);

        if (ros::Time::now() - start_time >= no_pose_timeout_ &&      // ensure we wait for the first pose
            ros::Time::now() - last_pose_time_ >= no_pose_timeout_)   // wait since the last pose received
        {
          result.outcome = thorp_msgs::FollowPoseResult::NO_POSE_TIMEOUT;
          follow_as_.setAborted(result);
          ROS_WARN("Following stopped: no pose received for %g seconds", no_pose_timeout_.toSec());
          break;
        }

        if (last_pose_time_.isZero())
        {
          ROS_WARN_THROTTLE(0.5, "Following waiting of the first pose to come (%g/%g seconds)",
                            (ros::Time::now() - start_time).toSec(), no_pose_timeout_.toSec());
        }
        else
        {
          // transform target pose into robot reference frame
          ttk::TF2::instance().transformPose(robot_frame_, target_pose_, target_pose_in_robot_frame);
          // Transform failure will print an appropriate error message, and the frame_id will be empty
        }
      }

      if (target_pose_in_robot_frame.header.frame_id == robot_frame_)
      {
        // we have valid target pose; calculate distance and heading to it

        double distance = ttk::distance2D(target_pose_in_robot_frame);
        double heading = ttk::heading(target_pose_in_robot_frame);

        if (goal->stop_at_distance && distance <= follow_distance)
        {
          result.outcome = thorp_msgs::FollowPoseResult::WITHIN_DISTANCE;
          follow_as_.setSucceeded(result);
          ROS_INFO("Following distance of %g m reached (current distance is %g m)", follow_distance, distance);
          break;
        }

        const geometry_msgs::Point& target_loc = target_pose_in_robot_frame.pose.position;
        if (std::abs(heading) > M_PI/2.0)
        {
          feedback_.cmd.angular.z = heading * w_scale_;
          ROS_INFO_THROTTLE(1, "Rotating at %.2f rad/s toward target %s, distance: %.2f m, heading: %.2f rad",
                            feedback_.cmd.angular.z, ttk::toCStr3D(target_loc), distance, heading);
        }
        else
        {
          feedback_.cmd.linear.x = v_scale_ * (distance - follow_distance);
          feedback_.cmd.angular.z = w_scale_ * heading;
          ROS_INFO_THROTTLE(1, "Following at %.2f m/s, %.2f rad/s target %s, distance: %.2f m, heading: %.2f rad",
                            feedback_.cmd.linear.x, feedback_.cmd.angular.z, ttk::toCStr3D(target_loc),
                            distance, heading);
        }
        feedback_.dist_to_target = (float)distance;
        feedback_.angle_to_target = (float)heading;
        feedback_.target_pose = target_pose_in_robot_frame;
        follow_as_.publishFeedback(feedback_);
        if (enabled_)
          twist_pub_.publish(feedback_.cmd);
      }

      if (!rate.sleep())
        ROS_WARN("Following loop took longer than the expected %g s... the loop actually took %g s",
                 rate.expectedCycleTime().toSec(), rate.cycleTime().toSec());
    }

    // Stop robot when done
    twist_pub_.publish(geometry_msgs::Twist());
  }
};

}  // thorp_navigation namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_follower");

  thorp::navigation::PoseFollower pf;
  pf.init();

  ros::spin();
}
