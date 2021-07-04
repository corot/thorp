/**
 * Subscribe to Gazebo topics and relay as ROS topics.
 */

#include <ros/ros.h>

#include <thorp_msgs/GraspEvent.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>

////#include <gazebo/msgs/grasp_event.pb.h>

gazebo::transport::SubscriberPtr gz_grasp_events_sub;

void eventCallback(gazebo::msgs::GraspEvent& event)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_topics_relay");

  // Gazebo side
//  gazebo::client::setup(argc, argv);
  // load gazebo server
  gazebo::setupServer(_argc, _argv);
  gazebo::transport::NodePtr gz_node(new gazebo::transport::Node());
  if (!gz_node->TryInit(gazebo::common::Time(60)))
  {
    ROS_ERROR("Gazebo global namespace was not found after 1 minute");
    return -1;
  }
  gz_grasp_events_sub = gz_node->Subscribe("gazebo/grasp_events", eventsCB);

  // ROS side
  ros::NodeHandle nh;
  events_pub = nh.advertise<thorp_msgs::GraspEvent>("gazebo/grasp_events", 1);

  ros::spin();
  gz_node->Fini();
  return 0;
}
