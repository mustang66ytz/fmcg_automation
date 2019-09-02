#ifndef MOVEIT_PLANNER_H
#define MOVEIT_PLANNER_H
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "rviz_simple_gui/MoveitPlanner.h"

class moveit_planner
{
public:
  moveit_planner();
  bool trigger_plan(rviz_simple_gui::MoveitPlanner::Request &req, rviz_simple_gui::MoveitPlanner::Response &res);
  bool plan_computation(rviz_simple_gui::MoveitPlanner::Request &req);

  ros::NodeHandle nh;

protected:
  bool result;
  std::string PLANNING_GROUP;
};

#endif // MOVEIT_PLANNER_H
