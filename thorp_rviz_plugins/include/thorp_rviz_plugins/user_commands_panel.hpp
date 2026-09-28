#pragma once

#include <vector>

#include <QHBoxLayout>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

namespace thorp_rviz_plugins
{
/**
 * Buttons that send commands to the running app, publishing them on a topic, user_command by default. The buttons are
 * set in the panel's RViz configuration, as a list of Name, Icon (a package:// URL) and Command, e.g.
 *
 *   Topic: /user_command
 *   Buttons:
 *     - Name: start
 *       Icon: package://thorp_bringup/rviz/icons/start.png
 *       Command: start
 */
class UserCommandsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit UserCommandsPanel(QWidget* parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

private:
  struct Button
  {
    QString name;
    QString icon;
    QString command;
  };

  void makePublisher();
  void makeButtons();
  void publish(const QString& command);

  QHBoxLayout* layout_;
  QString topic_ = "user_command";
  std::vector<Button> buttons_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace thorp_rviz_plugins
