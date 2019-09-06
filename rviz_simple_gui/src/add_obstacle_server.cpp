#include "ros/ros.h"
#include "rviz_simple_gui/planning_scene.h"
#include "rviz_simple_gui/AddObstacle.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_obstacle_server");
  planning_scene scene;
  ROS_INFO("Ready to add obstacles into the scene.");
  ros::ServiceServer service = scene.get_nh().advertiseService("add_obstacle", &planning_scene::trigger_plan, &scene);
  // using asynchronous spinner to prevent single thread resource occupying issue
  ros::spin();
  return 0;
}
