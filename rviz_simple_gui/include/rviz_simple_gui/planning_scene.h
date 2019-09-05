#ifndef PLANNING_SCENE_H
#define PLANNING_SCENE_H
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

// this class provides various planning scene objects and objects
class planning_scene
{
public:
  // class constructor
  planning_scene();
  // scene getter
  moveit::planning_interface::PlanningSceneInterface get_scene();
  // add collision block
  void add_object(std::string &block_id, int block_type, std::vector<double> dimension, std::vector<double> pose);
  // get collision blocks
  std::vector<moveit_msgs::CollisionObject> get_blocks();
protected:
  moveit::planning_interface::PlanningSceneInterface curr_scene;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
};

#endif // PLANNING_SCENE_H
