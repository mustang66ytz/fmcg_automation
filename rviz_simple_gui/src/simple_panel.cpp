#include "rviz_simple_gui/simple_panel.h"
#include <ros/console.h>
#include "rviz_simple_gui/simple_widget.h"
#include <QVBoxLayout>

rviz_simple_gui::SimplePanel::SimplePanel(QWidget* parent) : rviz::Panel(parent)
{
    ROS_INFO("Loaded simple RVIZ panel");
    QVBoxLayout* layout = new QVBoxLayout(this);
    widget_ = new SimpleWidget();
    layout->addWidget(widget_);
    setLayout(layout);
}

rviz_simple_gui::SimplePanel::~SimplePanel() {}

void rviz_simple_gui::SimplePanel::onInitialize()
{
    ROS_INFO("Initializng simple RVIZ panel");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_simple_gui::SimplePanel, rviz::Panel)
