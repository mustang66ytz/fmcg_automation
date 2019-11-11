#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

int main(int argc, char **argv){
    ros::init(argc, argv, "joint_state_visualizer_node");
    ros::NodeHandle n;
    ros::Publisher joints_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    
    
    ros::Rate r(100);
    
    // keep publishing the message
    while(ros::ok()){ 
        // initialize the joints to be published
        sensor_msgs::JointState mm_joints;
        mm_joints.header.seq = 0;
        mm_joints.header.stamp = ros::Time::now();
        mm_joints.header.frame_id = "";
        std::vector<std::string> joints = {"ur10_shoulder_pan_joint", "ur10_shoulder_lift_joint", "ur10_elbow_joint", "ur10_wrist_1_joint", "ur10_wrist_2_joint", "ur10_wrist_3_joint", "left_wheel_joint", "right_wheel_joint", "fl_caster_rotation_joint", "fl_caster_wheel_joint", "fr_caster_rotation_joint", "fr_caster_wheel_joint", "bl_caster_rotation_joint", "bl_caster_wheel_joint", "br_caster_rotation_joint", "br_caster_wheel_joint"};
        std::vector<double> joint_angles = {0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::cout<<"size:"<<joints.size()<<" size:"<<joint_angles.size();
        for(auto joint=joints.begin(); joint!=joints.end(); joint++){
            mm_joints.name.push_back(*joint);   
            std::cout<<"joint name: "<<*joint<<std::endl;
        }
        for(auto angle=joint_angles.begin(); angle!=joint_angles.end(); angle++){
            mm_joints.position.push_back(*angle);
            std::cout<<"joint angle: "<<*angle<<std::endl;
        }
        mm_joints.velocity = {};
        mm_joints.effort = {};
        
        joints_pub.publish(mm_joints);
        ros::spinOnce();
        r.sleep();
    }
}

