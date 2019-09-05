#include <ros/console.h> 
#include "rviz_simple_gui/simple_widget.h"  
#include "ui_simple_widget.h"
#include "rviz_simple_gui/MoveitPlanner.h"
#include "rviz_simple_gui/planning_scene.h"

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
    ROS_INFO_STREAM("Push Button A Clicked");
    ROS_INFO_STREAM("Create ROS Service client etc here to interface with external ROS services/topics etc");
}

void rviz_simple_gui::SimpleWidget::pushButton_B_clicked()
{
    ROS_INFO_STREAM("Push Button B Clicked");
    ROS_INFO_STREAM("Create ROS Service client etc here to interface with external ROS services/topics etc");
} 

void rviz_simple_gui::SimpleWidget::pushButton_C_clicked()
{
    ROS_INFO_STREAM("Push Button C Clicked");
    ROS_INFO_STREAM("Create ROS Service client etc here to interface with external ROS services/topics etc");
} 

// this function starts a ros service client requesting a motion planning from the ros service server
void rviz_simple_gui::SimpleWidget::pushButtonPlanning_clicked(){
    ROS_INFO_STREAM("Triggering the motion planning module");
    // initialize the motion planning client
    ros::ServiceClient client = nh_.serviceClient<rviz_simple_gui::MoveitPlanner>("motion_planning");
    rviz_simple_gui::MoveitPlanner srv;
    // define the target pose here
    std::vector<std::vector<double>> pose_candidates = {{0.0851781, -0.178383, 0.650531, 0.302129, -0.263213, 0.164935, 0.90124}, {-0.359662, 0.588864, 0.54586, 0.0790628, -0.532828, 0.387457, 0.748145}};
    geometry_msgs::Pose target_pose = setTarget(pose_candidates[0]);
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

void rviz_simple_gui::SimpleWidget::pushButtonAddCollisionOb_clicked(){
    ROS_INFO_STREAM("Adding collisiion objects into the scene");
    planning_scene scene;
    // add a sqaure box into the scene
    std::string block1_id = "front";
    int block1_type = 1; // 1 is for box
    std::vector<double> block1_dimension = {1, 0.1, 0.3};
    std::vector<double> block1_pose = {0, 0.5, 0.25};
    scene.add_object(block1_id, block1_type, block1_dimension, block1_pose);
    scene.get_scene().addCollisionObjects(scene.get_blocks());
}

