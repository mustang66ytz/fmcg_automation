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
  // constructor for the moveit_planner class
  moveit_planner();
  // this function will be executed when the service client is sending the request
  bool trigger_plan(rviz_simple_gui::MoveitPlanner::Request &req, rviz_simple_gui::MoveitPlanner::Response &res);
  // this function sets the target pose for the planner, and executes the plan, finally return the planning result
  bool plan_computation(rviz_simple_gui::MoveitPlanner::Request &req);
  // node handle accessor
  ros::NodeHandle get_nh(){
    return nh;
  }

protected:
  ros::NodeHandle nh;
  bool result;
  std::string PLANNING_GROUP;
};

#endif // MOVEIT_PLANNER_H
