#ifndef SIMPLE_GUI_WIDGET_H
#define SIMPLE_GUI_WIDGET_H

#include <QWidget>
#include <boost/assign/std/vector.hpp>
#include <iostream>

#include <ros/ros.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>
/*
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
*/

namespace Ui
{
class SimpleWidget;
}

namespace rviz_simple_gui
{ 
class SimpleWidget : public QWidget
{
  Q_OBJECT
public:
  SimpleWidget(QWidget* parent = 0);

  virtual ~SimpleWidget(); 

protected: 

protected Q_SLOTS:
  // Button Handlers 
  void pushButton_A_clicked(); 
  void pushButton_B_clicked();
  void pushButton_C_clicked(); 
  void pushButtonPlanning_clicked();
  void pushButtonAddCollisionOb_clicked();
  void sliderValue(int k);
protected:
  // UI
  Ui::SimpleWidget* ui_; 

  // ROS specific stuff
  ros::NodeHandle nh_;

};
}

#endif // SIMPLE_GUI_WIDGET_H
