/**
 * \file
 * \brief       RVIZ tool to set clear costmap
 * \author      Gautam <gautam.sharma@rapyuta-robotics.com>
 * \copyright   Copyright (c) 2021, Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include <thorp_rviz_plugins/button_tool.hpp>
#include <rviz/properties/string_property.h>
#include <std_srvs/Empty.h>
#include <memory.h>

namespace thorp::rviz_plugins
{
class ClearCostmapTool : public ButtonTool
{
  Q_OBJECT
public:
  ClearCostmapTool();
  ~ClearCostmapTool() = default;

  virtual void init() override;
  virtual void onClick() override;

protected Q_SLOTS:
  void updateService();

private:
  ros::ServiceClient clear_costmap_client_;
  std::unique_ptr<rviz::StringProperty> service_property_;
};

}  // namespace thorp::rviz_plugins
