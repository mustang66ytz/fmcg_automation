#include <ros/console.h> 
#include "rviz_simple_gui/simple_widget.h"  
#include "ui_simple_widget.h"


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

    // setup a ROS nodehandle (not required in blank rviz panel, just gere to demonstrate where objects go)
    ros::NodeHandle nh_;
}

rviz_simple_gui::SimpleWidget::~SimpleWidget()
{ 

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
}

