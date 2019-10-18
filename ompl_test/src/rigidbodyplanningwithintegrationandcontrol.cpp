#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "ompl_test/Waypoint.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
std::vector<double> path1d;
std::vector<std::vector<double>> path;
std::vector<std::vector<double>> control_signal;

// this class defines the kinematics car model (kinematics bicycle model)
class KinematicCarModel
 {
 public:
     // class constructor:  initialize the two private variables
     KinematicCarModel(const ob::StateSpace *space) : space_(space), carLength_(0.2)
     {
     }
     //  get the delta state updated
     void operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
     {
         const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
         const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

         dstate.resize(3);
         dstate[0] = u[0] * cos(theta);
         dstate[1] = u[0] * sin(theta);
         // inferred from the formula, the model used is kinematics bicycle model.
         // dstste[0] is the change of position in x direction
         // dstate[1] is the change of the position in y direction
         // dstate[2] is the change of the heading angle
         // u[0] is the linear velocity of the car
         // u[1] is the steering angle of the car
         dstate[2] = u[0] * tan(u[1]) / carLength_;
     }
     // this function update the state of the car after the control drive
     void update(ob::State *state, const std::valarray<double> &dstate) const
     {
         ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
         s.setX(s.getX() + dstate[0]);
         s.setY(s.getY() + dstate[1]);
         s.setYaw(s.getYaw() + dstate[2]);
         space_->enforceBounds(state);
     }

 private:
     // private variables: a pointer to a ompl::base::StateSpace and car length
     const ob::StateSpace *space_;
     const double          carLength_;

};
// this is a template class to propagate the state of a car under control input and a period of time
template<typename F>
 class EulerIntegrator
 {
 public:
     // class constructor with a space object pointer and time step
     EulerIntegrator(const ob::StateSpace *space, double timeStep) : space_(space), timeStep_(timeStep), ode_(space)
     {
     }
     // propagate the state with a control input within a period of time
     void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
     {
         double t = timeStep_;
         std::valarray<double> dstate;
         space_->copyState(result, start);
         while (t < duration + std::numeric_limits<double>::epsilon())
         {
             // every unit step actually calls the function defined in the KinematicCarModel class
             ode_(result, control, dstate); // calculate the dstate
             ode_.update(result, timeStep_ * dstate); // update the current state
             t += timeStep_;
         }
         // update one more time after the time exceeds the duration
         if (t + std::numeric_limits<double>::epsilon() > duration)
         {
             ode_(result, control, dstate);
             ode_.update(result, (t - duration) * dstate);
         }
     }

     double getTimeStep() const
     {
         return timeStep_;
     }

     void setTimeStep(double timeStep)
     {
         timeStep_ = timeStep;
     }

 private:
     const ob::StateSpace *space_;
     double                   timeStep_;
     F                        ode_;
 };

// check if the current state is valid
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
   //    ob::ScopedState<ob::SE2StateSpace>
   const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
   const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
   const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);
   // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
   return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:
    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
    {
    }
};

class DemoStatePropagator : public oc::StatePropagator
{
public:
    DemoStatePropagator(oc::SpaceInformation *si) : oc::StatePropagator(si),
                                                    integrator_(si->getStateSpace().get(), 0.0)
    {
    }
    // propagate the system from the current state to the future state
    void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override
    {
        integrator_.propagate(state, control, duration, result);
    }

    void setIntegrationTimeStep(double timeStep)
    {
        integrator_.setTimeStep(timeStep);
    }

    double getIntegrationTimeStep() const
    {
        return integrator_.getTimeStep();
    }

    EulerIntegrator<KinematicCarModel> integrator_;
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
 //print the solution path
 /*
for (int i=0; i<path.size(); i++){
    for(int j=0; j<3; j++){
        std::cout<<path[i][j]<<" ";
    }
    std::cout<<std::endl;
 }*/
 //print the control signals
 /*
for (int i=0; i<control_signal.size(); i++){
    for(int j=0; j<3; j++){
        std::cout<<control_signal[i][j]<<" ";
    }
    std::cout<<std::endl;
 }
 */
}

void planWithSimpleSetup()
{
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);
    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));
    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);
    cspace->setBounds(cbounds);
    // define a simple setup class
    oc::SimpleSetup ss(cspace);
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State *state) { return isStateValid(si, state); });
    auto propagator(std::make_shared<DemoStatePropagator>(si));
    ss.setStatePropagator(propagator);
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(0.0);
    goal->setY(0.5);
    goal->setYaw(0.0);
    ss.setStartAndGoalStates(start, goal, 0.05);
    ss.setup();
    propagator->setIntegrationTimeStep(si->getPropagationStepSize());
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
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
   }
   loop_rate.sleep();
 }
}

int main(int argc, char **argv)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    // Do motion planning
    planWithSimpleSetup();
    ros::init(argc, argv, "path_planner_node");
    // Visualize the result
    res_visualizer();
    return 0;
}
