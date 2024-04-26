#include <thorp_rviz_plugins/clear_costmap_tool.hpp>

namespace thorp::rviz_plugins
{
ClearCostmapTool::ClearCostmapTool()
{
  service_property_ = std::make_unique<rviz::StringProperty>(
      "Service", "/move_base_flex/clear_costmaps", "The service on which to make request for clearing costmaps.",
      getPropertyContainer(), SLOT(updateService()), this);
}

void ClearCostmapTool::init()
{
  setName("Clear Costmaps");
  updateService();
}

void ClearCostmapTool::updateService()
{
  try
  {
    clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>(service_property_->getStdString());
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("ClearCostmapTool", e.what());
  }
}

void ClearCostmapTool::onClick()
{
  std_srvs::Empty srv;
  if (clear_costmap_client_.call(srv))
  {
    ROS_INFO_NAMED("ClearCostmapTool", "Costmaps cleared!");
    setStatusMsg("Cleared the costmaps");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("ClearCostmapTool",
                           "Failed to call the service to clear costmaps: " << clear_costmap_client_.getService());
    setStatusMsg("Failed to clear the costmaps");
  }
}

}  // end namespace thorp::rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(thorp::rviz_plugins::ClearCostmapTool, rviz::Tool)
