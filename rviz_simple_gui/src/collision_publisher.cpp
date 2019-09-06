#include <ros/ros.h>
#include "rviz_simple_gui/planning_scene.h"

// this node utilizes the planning_scene to publish collision objects into the planning scene
int main(int argc, char **argv)
{
  // initialize ros node
  ros::init(argc, argv, "collision_publisher");
  // initialize a planning_scene object
  planning_scene scene;

  // add a sqaure box into the scene
  std::string block1_id = "front";
  int block1_type = 3; // 1 is for box
  std::vector<double> block1_dimension = {1, 0.2, 0.2};
  std::vector<double> block1_pose = {0, 0.5, 0.25};
  scene.add_object(block1_id, block1_type, block1_dimension, block1_pose);
  sleep(3);
  scene.get_scene().applyCollisionObjects(scene.get_blocks());
  ros::spin();
}
