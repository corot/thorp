#include "thorp_rviz_plugins/user_commands_panel.hpp"

#include <QLayoutItem>
#include <QToolButton>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>

namespace thorp_rviz_plugins
{
UserCommandsPanel::UserCommandsPanel(QWidget* parent) : rviz_common::Panel(parent), layout_(new QHBoxLayout(this))
{
}

void UserCommandsPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  makePublisher();
}

void UserCommandsPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  config.mapGetString("Topic", &topic_);
  buttons_.clear();
  const rviz_common::Config buttons = config.mapGetChild("Buttons");
  for (int i = 0; i < buttons.listLength(); ++i)
  {
    Button button;
    const rviz_common::Config entry = buttons.listChildAt(i);
    entry.mapGetString("Name", &button.name);
    entry.mapGetString("Icon", &button.icon);
    entry.mapGetString("Command", &button.command);
    buttons_.push_back(button);
  }
  makePublisher();
  makeButtons();
}

void UserCommandsPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_);
  rviz_common::Config buttons = config.mapMakeChild("Buttons");
  for (const auto& button : buttons_)
  {
    rviz_common::Config entry = buttons.listAppendNew();
    entry.mapSetValue("Name", button.name);
    entry.mapSetValue("Icon", button.icon);
    entry.mapSetValue("Command", button.command);
  }
}

void UserCommandsPanel::makePublisher()
{
  if (node_)
    publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_.toStdString(), 1);
}

void UserCommandsPanel::makeButtons()
{
  while (QLayoutItem* item = layout_->takeAt(0))
  {
    delete item->widget();
    delete item;
  }
  for (const auto& button : buttons_)
  {
    auto* tool_button = new QToolButton(this);
    tool_button->setText(button.name);
    tool_button->setToolTip(button.command);
    if (!button.icon.isEmpty())
    {
      tool_button->setIcon(rviz_common::loadPixmap(button.icon));
      tool_button->setIconSize(QSize(32, 32));
      tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    }
    const QString command = button.command;
    connect(tool_button, &QToolButton::clicked, this, [this, command]() { publish(command); });
    layout_->addWidget(tool_button);
  }
  layout_->addStretch();
}

void UserCommandsPanel::publish(const QString& command)
{
  if (!publisher_)
    return;
  std_msgs::msg::String message;
  message.data = command.toStdString();
  publisher_->publish(message);
}

}  // namespace thorp_rviz_plugins

PLUGINLIB_EXPORT_CLASS(thorp_rviz_plugins::UserCommandsPanel, rviz_common::Panel)
