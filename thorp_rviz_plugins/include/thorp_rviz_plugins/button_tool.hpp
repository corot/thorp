#pragma once

#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <jsk_rviz_plugins/OverlayText.h>

namespace thorp::rviz_plugins
{
/**
 * A base class for RViz tools implementing a simple button.
 *
 * The GUI button is released immediately upon click,
 * and the callback runs in the background on a different thread.
 */
class ButtonTool : public rviz::Tool
{
public:
  ButtonTool();
  virtual void onInitialize() final override;
  virtual void activate() final override;
  virtual void deactivate() final override;

protected:
  /**
   * Performs any setup needed after the display context has been set.
   */
  virtual void init() = 0;

  /**
   * Called when the button is clicked.
   */
  virtual void onClick() = 0;

  /**
   * Sets the status message that is displayed when the button is pressed.
   *
   * @param msg New status message.
   */
  void setStatusMsg(const std::string& status_msg);

protected:
  /** Handle to the public node. */
  ros::NodeHandle nh_;

  /** Handle to the private node. */
  ros::NodeHandle pnh_;

private:
  /** How long (sec) the status message is displayed for. */
  static constexpr double STATUS_MSG_LIFETIME_ = 1.5;

  /** Manager of this tool. */
  rviz::ToolManager* tool_manager_;

  /** Status message to display when the button is activated. */
  jsk_rviz_plugins::OverlayText status_msg_;

  /** Publisher for the status message. */
  ros::Publisher status_pub_;
};
}  // namespace thorp::rviz_plugins
