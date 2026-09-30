/**
 * \file
 * \brief       RViz tool to clear the costmaps
 * \author      Gautam <gautam.sharma@rapyuta-robotics.com>
 * \copyright   Copyright (c) 2021, Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

#include <nav2_msgs/srv/clear_entire_costmap.hpp>

namespace thorp_rviz_plugins
{
/**
 * Toolbar button clearing both Nav2 costmaps. It releases itself right after the click; the costmaps' answers are
 * logged when they arrive.
 */
class ClearCostmapTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  ClearCostmapTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

private:
  using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;

  std::vector<rclcpp::Client<ClearCostmap>::SharedPtr> clients_;
  rclcpp::Logger logger_;
};

}  // namespace thorp_rviz_plugins
