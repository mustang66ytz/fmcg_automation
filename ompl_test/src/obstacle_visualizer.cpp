#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "obstacle_visualizer");
  ros::NodeHandle node_handle;

  ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  visualization_msgs::Marker marker;
  marker.header.frame_id = "World";
  marker.header.stamp = ros::Time();
  marker.ns = "World";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  vis_pub.publish( marker );
  ros::Rate r(10);
  r.sleep();
  ros::spin();
}
