#include "ros/ros.h"
#include "rviz_simple_gui/MoveitPlanner.h"

bool moveit_planner(rviz_simple_gui::MoveitPlanner::Request &req, rviz_simple_gui::MoveitPlanner::Response &res){
  ROS_INFO("Planner executing!");
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_plan_server");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("motion_planning", moveit_planner);
  ROS_INFO("Ready to provide motion planning service");
  ros::spin();

  return 0;
}
