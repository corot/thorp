// ros
#include <ros/console.h>

// thorp_rviz_plugins
#include <thorp_rviz_plugins/cancel_navigation_tool.hpp>

namespace thorp::rviz_plugins
{
CancelNavigationTool::CancelNavigationTool()
{
  topic_property_ =
      new rviz::StringProperty("Topic", "/cancel_navigation", "The topic on which to publish cancel request.",
                               getPropertyContainer(), SLOT(updateTopic()), this);
  updateTopic();
  setStatusMsg("Canceled navigation");
}

void CancelNavigationTool::updateTopic()
{
  try
  {
    cancel_nav_pub_ = nh_.advertise<actionlib_msgs::GoalID>(topic_property_->getStdString(), 1, true);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("CancelNavigationTool", e.what());
  }
}

void CancelNavigationTool::init()
{
  setName("Cancel Navigation");
}

void CancelNavigationTool::onClick()
{
  cancel_nav_pub_.publish(actionlib_msgs::GoalID());
}

}  // end namespace thorp::rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(thorp::rviz_plugins::CancelNavigationTool, rviz::Tool)
