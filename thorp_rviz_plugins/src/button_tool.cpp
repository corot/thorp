#include <thorp_rviz_plugins/button_tool.hpp>
#include <rviz/display_context.h>
#include <thread>

namespace thorp::rviz_plugins
{
ButtonTool::ButtonTool() : nh_(), pnh_("~")
{
  status_pub_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("rviz/nav_status_overlay", 10);

  // msg style: white text on top-left corner
  status_msg_.width = 1000;
  status_msg_.height = 20;
  status_msg_.left = 5;
  status_msg_.top = 30;  // because we show robot name at 0
  status_msg_.text_size = 10;
  status_msg_.fg_color.r = status_msg_.fg_color.g = status_msg_.fg_color.b = status_msg_.fg_color.a = 1.0f;
}

void ButtonTool::onInitialize()
{
  tool_manager_ = context_->getToolManager();
  init();
}

void ButtonTool::activate()
{
  // this needs to be on a different thread
  // since the current thread is the render thread...
  std::thread(
      [&]()
      {
        // release the button
        tool_manager_->setCurrentTool(tool_manager_->getTool(0));

        // do stuff
        onClick();

        // show status msg
        status_msg_.action = jsk_rviz_plugins::OverlayText::ADD;
        status_pub_.publish(status_msg_);

        // hide status msg
        ros::Duration(STATUS_MSG_LIFETIME_).sleep();
        status_msg_.action = jsk_rviz_plugins::OverlayText::DELETE;
        status_pub_.publish(status_msg_);
      })
      .detach();
}

void ButtonTool::deactivate()
{
  // the button is deactivated immediately upon click
  // so subclasses don't need to implement this
}

void ButtonTool::setStatusMsg(const std::string& status_msg)
{
  status_msg_.text = status_msg;
}

}  // namespace thorp::rviz_plugins
