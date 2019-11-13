
#include "ompl_test/joint_state_visualizer.h"


bool JointStateVisualizer::resvis(ompl_test::ResultVisualizer::Request &req, ompl_test::ResultVisualizer::Response &res){
    ROS_INFO("Mobile manipulator trajectory visualization service is on!");
    // joint states publisher
    ros::Publisher joints_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    // tf broadcaster
    tf::TransformBroadcaster broadcaster;
    ros::Rate r(4);
    // define the name for all the joints
    std::vector<std::string> joints = {"ur10_shoulder_pan_joint", "ur10_shoulder_lift_joint", "ur10_elbow_joint", "ur10_wrist_1_joint", "ur10_wrist_2_joint", "ur10_wrist_3_joint", "left_wheel_joint", "right_wheel_joint", "fl_caster_rotation_joint", "fl_caster_wheel_joint", "fr_caster_rotation_joint", "fr_caster_wheel_joint", "bl_caster_rotation_joint", "bl_caster_wheel_joint", "br_caster_rotation_joint", "br_caster_wheel_joint"};
    // define the joint values (can be replaced by a 2d vector representing a trajectory)
    std::vector<std::vector<double>> path;
    std::vector<double> joint_angles;
    for(int i=0; i<req.traj_1d.size(); i++){
	if((i+1)%9 == 0){
	    joint_angles.push_back(req.traj_1d[i]);
	    path.push_back(joint_angles);
	    joint_angles.clear();
	}
	else{
	    joint_angles.push_back(req.traj_1d[i]);
	}
    }

    // tf message declaration
    sensor_msgs::JointState mm_joints;
    mm_joints.header.seq = 0;
    mm_joints.header.frame_id = "";
    for(auto joint=joints.begin(); joint!=joints.end(); joint++){
	mm_joints.name.push_back(*joint);   
    }
    // odom message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    // keep publishing the message
    while(ros::ok()){ 
	for(auto joint_angles: path){
	    mm_joints.header.stamp = ros::Time::now();
            mm_joints.position.clear();
	    for(auto angle=joint_angles.begin()+3; angle!=joint_angles.end(); angle++){
	        mm_joints.position.push_back(deg2rad(*angle));
	    }
            for(int i=0; i<10; i++){
	        mm_joints.position.push_back(0);
	    }
	    mm_joints.velocity = {};
	    mm_joints.effort = {};
	    // update transform
	    odom_trans.header.stamp = ros::Time::now();
	    odom_trans.transform.translation.x = joint_angles[0];
	    odom_trans.transform.translation.y = joint_angles[1];
	    odom_trans.transform.translation.z = 0.0;
	    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(joint_angles[2]);
	    //send the joint state and transform
	    broadcaster.sendTransform(odom_trans);
	    joints_pub.publish(mm_joints);
	    ros::spinOnce();
	    r.sleep();
	}
    }
    res.success = true;
    return true;
}
