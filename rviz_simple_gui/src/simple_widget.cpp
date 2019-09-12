#include <ros/console.h> 
#include "rviz_simple_gui/simple_widget.h"  
#include "ui_simple_widget.h"
#include "rviz_simple_gui/MoveitPlanner.h"
#include "rviz_simple_gui/planning_scene.h"
#include "rviz_simple_gui/AddObstacle.h"
#include <vector>

rviz_simple_gui::SimpleWidget::SimpleWidget(QWidget* parent)
    : QWidget(parent) 
{
    // UI setup
    ui_ = new Ui::SimpleWidget;
    ui_->setupUi(this);

    // Wire in buttons
    connect(ui_->pushButton_A, SIGNAL(clicked()), this, SLOT(pushButton_A_clicked()));
    connect(ui_->pushButton_B, SIGNAL(clicked()), this, SLOT(pushButton_B_clicked()));
    connect(ui_->pushButton_C, SIGNAL(clicked()), this, SLOT(pushButton_C_clicked()));
    connect(ui_->pushButtonPlanning, SIGNAL(clicked()), this, SLOT(pushButtonPlanning_clicked()));
    connect(ui_->pushButtonAddCollisionOb, SIGNAL(clicked()), this, SLOT(pushButtonAddCollisionOb_clicked()));

    // setting up the horizontal slider bar
    ui_->obstacleX->setRange(-20, 20);
    ui_->obstacleX->setSingleStep(1);
    ui_->obstacleY->setRange(-20, 20);
    ui_->obstacleY->setSingleStep(1);
    ui_->obstacleZ->setRange(-20, 20);
    ui_->obstacleZ->setSingleStep(1);
    connect(ui_->obstacleX, SIGNAL(valueChanged(int)), this, SLOT(sliderValueX(int)));
    connect(ui_->obstacleY, SIGNAL(valueChanged(int)), this, SLOT(sliderValueY(int)));
    connect(ui_->obstacleZ, SIGNAL(valueChanged(int)), this, SLOT(sliderValueZ(int)));
    // setup a ROS nodehandle (not required in blank rviz panel, just gere to demonstrate where objects go)
    ros::NodeHandle nh_;
}

rviz_simple_gui::SimpleWidget::~SimpleWidget()
{ 

}

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

void rviz_simple_gui::SimpleWidget::pushButton_A_clicked()
{
    ROS_INFO("inside box pose is selected");
    this->target_id = 0;
}

void rviz_simple_gui::SimpleWidget::pushButton_B_clicked()
{
  ROS_INFO("home pose is selected");
  this->target_id = 1;
} 

void rviz_simple_gui::SimpleWidget::pushButton_C_clicked()
{
  ROS_INFO("up-straight pose is selected");
  this->target_id = 2;
} 

// this function starts a ros service client requesting a motion planning from the ros service server
void rviz_simple_gui::SimpleWidget::pushButtonPlanning_clicked(){
    ROS_ERROR("Triggering the motion planning module");
    // initialize the motion planning client
    ros::ServiceClient client = nh_.serviceClient<rviz_simple_gui::MoveitPlanner>("motion_planning");
    rviz_simple_gui::MoveitPlanner srv;
    // define the target pose here
    std::vector<double> home_pose = {1.18431, 0.256129, 1.06171, 3.41224e-05, 0.707126, 0.707087, 3.41215e-05};
    std::vector<double> up_pose = {0.000207902, 0.256141, 2.4773, 0.707103, 0.000142007, 0.000142024, 0.707111};
    std::vector<std::vector<double>> pose_candidates = {{0.808644, 0.192714, 0.993994, 0.000165724, -0.707075, -0.707139, -9.5012e-05}, home_pose, up_pose};
    geometry_msgs::Pose target_pose = setTarget(pose_candidates[target_id]);
    srv.request.target = target_pose;
    if (client.call(srv))
    {
      ROS_INFO("Sum: %d", srv.response.success);
    }
    else
    {
      ROS_ERROR("Failed to call service motion_plan_server");
    }
}

void rviz_simple_gui::SimpleWidget::addObjectPrimitive(std::string &block_id, int block_type, std::vector<double> block_dimension,
                                                       std::vector<double> block_pose){
    // initialize the add obstacle client
    ros::ServiceClient client = nh_.serviceClient<rviz_simple_gui::AddObstacle>("add_obstacle");
    rviz_simple_gui::AddObstacle srv;

    // build the square box based on the position provided:
    srv.request.block_id = block_id;
    srv.request.block_type = block_type;
    srv.request.block_dimension = block_dimension;
    srv.request.block_pose = block_pose;
    if (client.call(srv))
    {
      ROS_INFO("Successfully added");
    }
    else
    {
      ROS_ERROR("Failed to call service add_obstacle_server");
    }
}

void rviz_simple_gui::SimpleWidget::pushButtonAddCollisionOb_clicked(){
    ROS_ERROR("Adding collisiion objects into the scene");
    std::vector<std::string> block_id_group = {"front", "back", "left", "right", "bottom"};
    for (auto block_id: block_id_group){
      if (block_id == "front"){
        addObjectPrimitive(block_id, 1, {0.1, 1, 0.3}, {x_position+0.5, y_position, z_position, 1});
      }
      if (block_id == "back"){
        addObjectPrimitive(block_id, 1, {0.1, 1, 0.3}, {x_position-0.5, y_position, z_position, 1});
      }
      if (block_id == "left"){
        addObjectPrimitive(block_id, 1, {1, 0.1, 0.3}, {x_position, y_position+0.5, z_position, 1});
      }
      if (block_id == "right"){
        addObjectPrimitive(block_id, 1, {1, 0.1, 0.3}, {x_position, y_position-0.5, z_position, 1});
      }
      if (block_id == "bottom"){
        addObjectPrimitive(block_id, 1, {1, 1, 0.1}, {x_position, y_position, z_position-0.1, 1});
      }
    }
}

void rviz_simple_gui::SimpleWidget::sliderValueX(int k){
    double k_ = k/10.0;
    ROS_INFO("updating x position to: %f", k_);
    x_position = k_;
}

void rviz_simple_gui::SimpleWidget::sliderValueY(int k){
    double k_ = k/10.0;
    ROS_INFO("updating y position to: %f", k_);
    y_position = k_;
}

void rviz_simple_gui::SimpleWidget::sliderValueZ(int k){
    double k_ = k/10.0;
    ROS_INFO("updating z position to: %f", k_);
    z_position = k_;
}
