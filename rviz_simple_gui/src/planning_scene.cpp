#include "rviz_simple_gui/planning_scene.h"
#include <vector>
// class constructor
planning_scene::planning_scene()
{
  // initialize a node handler
  ros::NodeHandle nh;
  // initialize a planning scene object
  moveit::planning_interface::PlanningSceneInterface curr_scene;
  sleep(5);
}

// scene getter
moveit::planning_interface::PlanningSceneInterface planning_scene::get_scene(){
  return curr_scene;
}

// add collision block
void planning_scene::add_object(std::string &block_id, int block_type, std::vector<double> dimension, std::vector<double> block_pose){
  moveit_msgs::CollisionObject block;
  block.id = block_id;
  shape_msgs::SolidPrimitive primitive;
  // block type: 1:box, 2:sphere, 3:cylinder, 4:cone
  switch(block_type){
    case 1:{
      std::cout<<"adding a box to the scene"<<std::endl;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      break;
    }
    case 2:{
      std::cout<<"adding a sphere to the scene"<<std::endl;
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(1);
      break;
    }
    case 3:{
      std::cout<<"adding a cylinder to the scene"<<std::endl;
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(3);
      break;
    }
    case 4:{
      std::cout<<"adding a cone to the scene"<<std::endl;
      primitive.dimensions.resize(2);
      break;
    }
  }
  primitive.dimensions[0] = dimension[0];
  primitive.dimensions[1] = dimension[1];
  primitive.dimensions[2] = dimension[2];
  // define the pose of the collision object
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.position.x = block_pose[0];
  pose.position.y = block_pose[1];
  pose.position.z = block_pose[2];

  block.primitives.push_back(primitive);
  block.primitive_poses.push_back(pose);
  block.operation = block.ADD;

  // add the collision object to the list of collision objects
  collision_objects.push_back(block);
}

// get collision blocks
std::vector<moveit_msgs::CollisionObject> planning_scene::get_blocks(){
  return collision_objects;
}


