#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "rviz_simple_gui/moveit_planner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_plan_server");
  //ros::NodeHandle nh;
  // initialize a moveit_planner object
  moveit_planner planner;
  ros::ServiceServer service = planner.nh.advertiseService("motion_planning", &moveit_planner::trigger_plan, &planner);
  ROS_INFO("Ready to provide motion planning service");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  //ros::spin();

  return 0;
}
