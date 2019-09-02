#include "rviz_simple_gui/moveit_planner.h"

moveit_planner::moveit_planner()
{
  //ros::NodeHandle nh;
  //Move group setup
  this->PLANNING_GROUP = "manipulator";
}

bool moveit_planner::trigger_plan(rviz_simple_gui::MoveitPlanner::Request &req, rviz_simple_gui::MoveitPlanner::Response &res){
  ROS_INFO("Planner executing!");
  res.success = this->plan_computation(req);
  return res.success;
}

bool moveit_planner::plan_computation(rviz_simple_gui::MoveitPlanner::Request &req){
  moveit::planning_interface::MoveGroupInterface object(this->PLANNING_GROUP);
  ros::Publisher display_publisher = this->nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  // moveit planning stage
  object.setPoseTarget(req.target);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = object.plan(my_plan);
  if (success.val == 1){
    return true;
  }
  else{
    return 0;
  }
}
