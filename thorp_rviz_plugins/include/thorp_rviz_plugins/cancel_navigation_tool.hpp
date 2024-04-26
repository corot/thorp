/**
 *  \file
 *  \brief      Cancel navigation
 *  \author     Renan <renan.salles@rapyuta-robotics.com>
 *  \copyright  Copyright (c) 2021, Rapyuta Robotics Co., Ltd.
 */

#pragma once

// ros
#include <thorp_rviz_plugins/button_tool.hpp>
#include <rviz/properties/string_property.h>
#include <actionlib/client/simple_action_client.h>

namespace thorp::rviz_plugins
{
class CancelNavigationTool : public ButtonTool
{
  Q_OBJECT
public:
  CancelNavigationTool();
  ~CancelNavigationTool() = default;

protected Q_SLOTS:
  void updateTopic();
  virtual void init() override;
  virtual void onClick() override;

private:
  ros::Publisher cancel_nav_pub_;
  rviz::StringProperty* topic_property_;
};

}  // namespace thorp::rviz_plugins
