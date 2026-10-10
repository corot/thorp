#include "thorp_rviz_plugins/clear_costmap_tool.hpp"

#include <string>

#include <QTimer>

#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>

namespace thorp_rviz_plugins
{
ClearCostmapTool::ClearCostmapTool() : logger_(rclcpp::get_logger("clear_costmap_tool"))
{
}

void ClearCostmapTool::onInitialize()
{
  setName("Clear Costmaps");
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  logger_ = node->get_logger();
  for (const char* service :
       { "/global_costmap/clear_entirely_global_costmap", "/local_costmap/clear_entirely_local_costmap" })
    clients_.push_back(node->create_client<ClearCostmap>(service));
}

void ClearCostmapTool::activate()
{
  for (const auto& client : clients_)
  {
    const std::string service = client->get_service_name();
    if (!client->service_is_ready())
    {
      RCLCPP_ERROR(logger_, "Can't clear the costmap: service %s not available", service.c_str());
      continue;
    }
    // rclcpp matches client callbacks by their exact parameter types
    client->async_send_request(std::make_shared<ClearCostmap::Request>(),
                               // NOLINTNEXTLINE(performance-unnecessary-value-param)
                               [this, service](rclcpp::Client<ClearCostmap>::SharedFuture)
                               { RCLCPP_INFO(logger_, "Costmap cleared by %s", service.c_str()); });
  }
  // A button, not a mode: go back to the default tool once the tool manager has finished activating this one
  QTimer::singleShot(0, this,
                     [this]()
                     {
                       auto tool_manager = context_->getToolManager();
                       tool_manager->setCurrentTool(tool_manager->getDefaultTool());
                     });
}

void ClearCostmapTool::deactivate()
{
}

}  // namespace thorp_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(thorp_rviz_plugins::ClearCostmapTool, rviz_common::Tool)
