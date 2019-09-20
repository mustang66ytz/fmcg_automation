#include "rviz_simple_gui/moveit_planner.h"

// constructor for the moveit_planner class
moveit_planner::moveit_planner()
{
  //Move group setup
  this->PLANNING_GROUP = "manipulator";
}

// this function will be executed when the service client is sending the request
bool moveit_planner::trigger_plan(rviz_simple_gui::MoveitPlanner::Request &req, rviz_simple_gui::MoveitPlanner::Response &res){
  ROS_INFO("Planner executing!");
  // call the plan_execution function for the moveit path planning
  res.success = this->plan_computation(req);
  this->visualization();
  return res.success;
}

bool moveit_planner::visualization(){
  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base","/rviz_visual_markers"));
  // Create pose
  Eigen::Isometry3d pose;
  pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
  pose.translation() = Eigen::Vector3d( 0.1, 0.1, 3 ); // translate x,y,z
  // Publish arrow vector of pose
  ROS_ERROR("Visualizing the markers!");
  visual_tools_->publishArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);

  // Don't forget to trigger the publisher!
  visual_tools_->trigger();
  return true;
}

// this function sets the target pose for the planner, and executes the plan, finally return the planning result
bool moveit_planner::plan_computation(rviz_simple_gui::MoveitPlanner::Request &req){
  moveit::planning_interface::MoveGroupInterface object(this->PLANNING_GROUP);
  ros::Publisher display_publisher = this->nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  // moveit planning stage
  object.setPoseTarget(req.target);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = object.plan(my_plan);
  if (success.val == 1){
    ROS_INFO("Executing the plan now!");
    // moveit execution stage
    object.move();
    return true;
  }
  else{
    return 0;
  }
}
