#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// This function return a target pose from an input pose with vector format
geometry_msgs::Pose setTarget(std::vector<double> target_pose){
  geometry_msgs::Pose target;
  target.orientation.w = target_pose[3];
  target.orientation.x = target_pose[4];
  target.orientation.y = target_pose[5];
  target.orientation.z = target_pose[6];
  target.position.x = target_pose[0];
  target.position.y = target_pose[1];
  target.position.z = target_pose[2];
  return target;
}

int main(int argc, char **argv)
{
  //ROS initialization
  ros::init(argc, argv, "custom_path");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);

  //Move group setup
  static const std::string PLANNING_GROUP_1 = "arm";
  moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP_1);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  //Target pose setup
  // A random valid end-effector pose (-0.359662 0.588864 0.54586 0.0790628 -0.532828 0.387457 0.748145)
  // A valid end-effector pose when obstacle is present: 0.0851781 -0.178383 0.650531 0.302129 -0.263213 0.164935 0.90124
  std::vector<std::vector<double>> pose_candidates = {{0.0851781, -0.178383, 0.650531, 0.302129, -0.263213, 0.164935, 0.90124}, {-0.359662, 0.588864, 0.54586, 0.0790628, -0.532828, 0.387457, 0.748145}};
  geometry_msgs::Pose target_pose1 = setTarget(pose_candidates[0]);
  arm.setPoseTarget(target_pose1);

  //moveit planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = arm.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s", success.val? "":"Failed");
  sleep(5);
  ros::shutdown();
}
