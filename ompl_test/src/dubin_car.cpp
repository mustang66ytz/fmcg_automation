#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include "ompl_test/Waypoint.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
std::vector<double> path1d;
std::vector<std::vector<double>> path;
std::vector<std::vector<double>> control_signal;

// Definition of the ODE for the kinematic car.
// This method is analogous to the above KinematicCarModel::ode function.
void KinematicCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    double carLength = 0.2;

    // Zero out qdot
    qdot.resize (q.size (), 0);
    /*
    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(u[1]) / carLength;
    */
    /* moderate kinematics bicycle model*/
    // define the rear and front lengths of the car using bicycle model
    double lr, lf = carLength/2.0;
    double beta = atan((lr/carLength)*tan(u[1]));
    qdot[0] = u[0]*cos(theta+beta);
    qdot[1] = u[0]*sin(theta+beta);
    qdot[2] = u[0]*sin(beta)/lr;
}

// This is a callback method invoked after numerical integration.
void KinematicCarPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);
    double x = state->as<ob::SE2StateSpace::StateType>()->getX();
    double y = state->as<ob::SE2StateSpace::StateType>()->getY();
    double yaw = state->as<ob::SE2StateSpace::StateType>()->getYaw();
    // obstacle avoidance
    bool no_collision = true;
    if(x>=0.3 && x<=0.7 && y>=0.3 && y<=0.7){
      no_collision = false;
    }
    std::cout<<"no_collision?"<<no_collision<<std::endl;
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && no_collision && (const void*)rot != (const void*)pos;
}

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:

    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
    {
    }
};

// this function processes the path planned
void resultProcessor(oc::SimpleSetup &ss){
 oc::PathControl solution_path = ss.getSolutionPath();
 std::vector<ob::State* >& states = solution_path.getStates();
 std::vector<oc::Control* >& controls = solution_path.getControls();

 // test the result construction
 for(int i=0; i!=states.size(); i++){
     std::vector<double> row;
     double x = states[i]->as<ob::SE2StateSpace::StateType>()->getX();
     double y = states[i]->as<ob::SE2StateSpace::StateType>()->getY();
     double yaw = states[i]->as<ob::SE2StateSpace::StateType>()->getYaw();
     if(i<states.size()-1){
       std::vector<double> control_row;
       double x_control = controls[i]->as<oc::RealVectorControlSpace::ControlType>()->values[0];
       double y_control = controls[i]->as<oc::RealVectorControlSpace::ControlType>()->values[1];
       double yaw_control = controls[i]->as<oc::RealVectorControlSpace::ControlType>()->values[2];
       control_row.push_back(x_control);
       control_row.push_back(y_control);
       control_row.push_back(yaw_control);
       control_signal.push_back(control_row);
       //std::cout<<"control x:"<<x_control<<std::endl;
     }

     row.push_back(x);
     row.push_back(y);
     row.push_back(yaw);
     path.push_back(row);
     path1d.push_back(x);
     path1d.push_back(y);
     path1d.push_back(yaw);
 }

 for (int i=0; i<3; i++){
  std::cout<<"pos at time 0: "<<path[0][i]<<std::endl;
 }
 for (int i=0; i<3; i++){
  std::cout<<"control at time 0: "<<control_signal[0][i]<<std::endl;
 }
 for (int i=0; i<3; i++){
  std::cout<<"pos at time 1: "<<path[1][i]<<std::endl;
 }
}

void planWithSimpleSetup()
{
    auto space(std::make_shared<ob::SE2StateSpace>());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, 0.0);
    cbounds.setHigh(0, 0.3);
    cbounds.setLow(1, -1);
    cbounds.setHigh(1, 1);
    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker([si](const ob::State *state) { return isStateValid(si, state); });

    // Setting the propagation routine for this space:
    // KinematicCarModel does NOT use ODESolver
    //ss.setStatePropagator(std::make_shared<KinematicCarModel>(ss.getSpaceInformation()));

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &KinematicCarODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));
    si->setMinMaxControlDuration(1,20);
    si->setPropagationStepSize(0.1); // decrease this value to get a finer path
    ss.setPlanner(std::make_shared<oc::RRT>(ss.getSpaceInformation()));

    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
    start->setYaw(0.0);

    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(0.5);
    goal->setY(1);
    goal->setYaw(0.0);

    ss.setStartAndGoalStates(start, goal, 0.1);

    ss.setup();

    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        oc::PathControl& path(ss.getSolutionPath());
        path.interpolate();
        resultProcessor(ss);
        //ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

// the function is responsible for visualizing the planned path and drive the car accordingly
void res_visualizer(){
 // publish the path and the environment to the visualizer
 ros::NodeHandle n;
 ros::Publisher path_pub = n.advertise<ompl_test::Waypoint>("trajectory_pose", 100);
 ros::Rate loop_rate(10);
 while(ros::ok()){
   ompl_test::Waypoint msg;
   msg.target = path1d;
   ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
   ros::Publisher obstacle_pub = n.advertise<geometry_msgs::PolygonStamped>("obstacle", 50);
   tf::TransformBroadcaster odom_broadcaster;
   ros::Rate r(5);
   // iterate through the result states
   std::cout<<"publishing the trajectory states to odom"<<std::endl;
   for(int i=0; i<path.size(); i++){
     path_pub.publish(msg);
     ros::Time current_time = ros::Time::now();
     double x = path[i][0]; // define the x position
     double y = path[i][1]; // define the y position
     double yaw = path[i][2]; //define the yaw
     //since all odometry is 6DOF we'll need a quaternion created from yaw
     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

     //first, we'll publish the transform over tf
     geometry_msgs::TransformStamped odom_trans;
     odom_trans.header.stamp = current_time;
     odom_trans.header.frame_id = "odom";
     odom_trans.child_frame_id = "base_link";
     odom_trans.transform.translation.x = x;
     odom_trans.transform.translation.y = y;
     odom_trans.transform.translation.z = 0.0;
     odom_trans.transform.rotation = odom_quat;
     //send the transform
     odom_broadcaster.sendTransform(odom_trans);

     //next, we'll publish the odometry message over ROS
     nav_msgs::Odometry odom;
     odom.header.stamp = current_time;
     odom.header.frame_id = "odom";
     //set the position
     odom.pose.pose.position.x = x;
     odom.pose.pose.position.y = y;
     odom.pose.pose.position.z = 0.0;
     odom.pose.pose.orientation = odom_quat;
     //set the velocity
     odom.child_frame_id = "base_link";
     odom.twist.twist.linear.x = 0;
     odom.twist.twist.linear.y = 0;
     odom.twist.twist.angular.z = 0;
     //publish the message
     odom_pub.publish(odom);
     r.sleep();

     // obstacle publisher
     geometry_msgs::PolygonStamped obstacle;
     obstacle.header.stamp = ros::Time::now();
     obstacle.header.frame_id = "World";
     // set the position
     geometry_msgs::Point32 pt_a;
     pt_a.x = 0.3;
     pt_a.y = 0.3;
     pt_a.z = 0;
     geometry_msgs::Point32 pt_b;
     pt_b.x = 0.3;
     pt_b.y = 0.7;
     pt_b.z = 0;
     geometry_msgs::Point32 pt_c;
     pt_c.x = 0.7;
     pt_c.y = 0.7;
     pt_c.z = 0;
     geometry_msgs::Point32 pt_d;
     pt_d.x = 0.7;
     pt_d.y = 0.3;
     pt_d.z = 0;
     obstacle.polygon.points.push_back(pt_a);
     obstacle.polygon.points.push_back(pt_b);
     obstacle.polygon.points.push_back(pt_c);
     obstacle.polygon.points.push_back(pt_d);
     //publish the message
     obstacle_pub.publish(obstacle);
     r.sleep();
   }
   loop_rate.sleep();
 }
}

int main(int argc, char ** argv)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    planWithSimpleSetup();
    ros::init(argc, argv, "path_planner_node");
    // Visualize the result
    res_visualizer();
    return 0;
}
