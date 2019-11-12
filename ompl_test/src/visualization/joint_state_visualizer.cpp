#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <vector>

int main(int argc, char **argv){
    // ros initialization
    ros::init(argc, argv, "joint_state_visualizer_node");
    ros::NodeHandle n;
    // joint states publisher
    ros::Publisher joints_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    // tf broadcaster
    tf::TransformBroadcaster broadcaster;
    ros::Rate r(50);
    // define the name for all the joints
    std::vector<std::string> joints = {"ur10_shoulder_pan_joint", "ur10_shoulder_lift_joint", "ur10_elbow_joint", "ur10_wrist_1_joint", "ur10_wrist_2_joint", "ur10_wrist_3_joint", "left_wheel_joint", "right_wheel_joint", "fl_caster_rotation_joint", "fl_caster_wheel_joint", "fr_caster_rotation_joint", "fr_caster_wheel_joint", "bl_caster_rotation_joint", "bl_caster_wheel_joint", "br_caster_rotation_joint", "br_caster_wheel_joint"};
    // define the joint values (can be replaced by a 2d vector representing a trajectory)
    std::vector<double> joint_angles = {0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double angle = 0;

    // tf message declaration
    sensor_msgs::JointState mm_joints;
    mm_joints.header.seq = 0;
    mm_joints.header.frame_id = "";
    // odom message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
 
    // keep publishing the message
    while(ros::ok()){ 
        mm_joints.header.stamp = ros::Time::now();

        for(auto joint=joints.begin(); joint!=joints.end(); joint++){
            mm_joints.name.push_back(*joint);   
        }
        for(auto angle=joint_angles.begin(); angle!=joint_angles.end(); angle++){
            mm_joints.position.push_back(*angle);
        }
        mm_joints.velocity = {};
        mm_joints.effort = {};

        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
        
        //send the joint state and transform
        broadcaster.sendTransform(odom_trans);
        joints_pub.publish(mm_joints);
        
        // state update happens here
        angle += 0.1;
        ros::spinOnce();
        r.sleep();
    }
}

