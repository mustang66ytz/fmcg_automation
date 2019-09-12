#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_listener.h>

// this function monitors the arm's end-effector pose
void monitorEE(std::string& PLANNING_GROUP){
//Move group setup
moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  while (ros::ok()){
    geometry_msgs::PoseStamped current_raw_pose = group.getCurrentPose(group.getEndEffectorLink().c_str());
    // Parse the current end-effector position
    std::vector<double> current_position;
    double current_position_x = current_raw_pose.pose.position.x;
    double current_position_y = current_raw_pose.pose.position.y;
    double current_position_z = current_raw_pose.pose.position.z;
    double current_orientation_x = current_raw_pose.pose.orientation.x;
    double current_orientation_y = current_raw_pose.pose.orientation.y;
    double current_orientation_z = current_raw_pose.pose.orientation.z;
    double current_orientation_w = current_raw_pose.pose.orientation.w;

    current_position.push_back(current_position_x);
    current_position.push_back(current_position_y);
    current_position.push_back(current_position_z);
    current_position.push_back(current_orientation_w);
    current_position.push_back(current_orientation_x);
    current_position.push_back(current_orientation_y);
    current_position.push_back(current_orientation_z);
    ROS_INFO("The current pose is: ");
    for(auto const& i:current_position){
      std::cout <<i<<" ";
    }
    sleep(1);
  }
}

// this function monitors the arm's joint positions
void monitorArm(std::string& PLANNING_GROUP){
  //Move group setup
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  while (ros::ok()){
    robot_state::RobotStatePtr current_state = group.getCurrentState();
    // Parse the current end-effector position
    std::vector<double> joint_positions;
    current_state->copyJointGroupPositions(PLANNING_GROUP, joint_positions);
    ROS_INFO("The current configuration is: ");
    for(auto const& i:joint_positions){
      std::cout <<i<<" ";
    }
    sleep(1);
  }
}

int main(int argc, char **argv)
{
  //ROS initialization
  ros::init(argc, argv, "position_tracker");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);
  // get user input for position tracking for the group of interest
  // 1: end-effector 2: arm joints 3: base
  int selection = 1;

  static std::string PLANNING_GROUP;
  if (selection == 1){
    PLANNING_GROUP = "manipulator";
    monitorEE(PLANNING_GROUP);
  }
  ROS_INFO("Finished!");
}
