/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


 #include <ompl/base/goals/GoalState.h>
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/base/spaces/DiscreteStateSpace.h>
 #include <ompl/control/spaces/RealVectorControlSpace.h>
 #include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include "ompl_test/joint_state_visualizer.h"
#include <ompl_test/ResultVisualizer.h>
 #include <ompl/config.h>
 #include <iostream>
 #include <limits>
 #include <boost/math/constants/constants.hpp>

 namespace ob = ompl::base;
 namespace oc = ompl::control;

 void propagate(const oc::SpaceInformation *si, const ob::State *state,
     const oc::Control* control, const double duration, ob::State *result)
 {
     static double timeStep = .01;
     int nsteps = ceil(duration / timeStep);
     double dt = duration / nsteps;
     const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

     ob::CompoundStateSpace::StateType& s = *result->as<ob::CompoundStateSpace::StateType>();
     ob::SE2StateSpace::StateType& se2 = *s.as<ob::SE2StateSpace::StateType>(0);
     ob::RealVectorStateSpace::StateType& angles = *s.as<ob::RealVectorStateSpace::StateType>(1);

     si->getStateSpace()->copyState(result, state);
     for(int i=0; i<nsteps; i++)
     {
         // propagate the joint angle
         angles.values[0] = angles.values[0] + dt*u[2];
         angles.values[1] = angles.values[1] + dt*u[3];
         angles.values[2] = angles.values[2] + dt*u[4];
         angles.values[3] = angles.values[3] + dt*u[5];
         angles.values[4] = angles.values[4] + dt*u[6];
         angles.values[5] = angles.values[5] + dt*u[7];
         se2.setX(se2.getX() + dt * u[1] * cos(se2.getYaw()));
         se2.setY(se2.getY() + dt * u[1] * sin(se2.getYaw()));
         se2.setYaw(se2.getYaw() + dt * u[0]);
         if (!si->satisfiesBounds(result))
             return;
     }
 }

 // The free space consists of two narrow corridors connected at right angle.
 // To make the turn, the car will have to downshift.
 bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
 {
   const auto *se2 = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
   return si->satisfiesBounds(state) && (se2->getX() < -80. || se2->getY() > 80.);
 }


 int main(int argc, char** argv)
 {
     // plan for hybrid car in SE(2) with discrete gears
     auto SE2(std::make_shared<ob::SE2StateSpace>());
     auto Joints(std::make_shared<ob::RealVectorStateSpace>(6));
     ob::StateSpacePtr stateSpace = SE2 + Joints;

     // set the bounds for the R^2 part of SE(2)
     ob::RealVectorBounds bounds(2);
     bounds.setLow(0, -90);
     bounds.setHigh(0, 100);
     bounds.setLow(1, -100);
     bounds.setHigh(1, 90);
     //bounds.setLow(2, -boost::math::constants::pi<double>());
     //bounds.setHigh(2, boost::math::constants::pi<double>());
     SE2->setBounds(bounds);

     // set the bounds for all the joints
     ob::RealVectorBounds angleBound(6);
     angleBound.setLow(-60);
     angleBound.setHigh(60);
     Joints->setBounds(angleBound);

     // create start and goal states
     ob::ScopedState<> start(stateSpace);
     ob::ScopedState<> goal(stateSpace);

     // Both start and goal are states with high velocity with the car in third gear.
     // However, to make the turn, the car cannot stay in third gear and will have to
     // shift to first gear.
     start[0] = start[1] = -90.; // position
     start[2] = boost::math::constants::pi<double>()/2; // orientation
     start[3] = 40.; // joint angle
     start[4] = 0.; // joint angle
     start[5] = 0.; // joint angle
     start[6] = 0.; // joint angle
     start[7] = 0.; // joint angle
     start[8] = 0.; // joint angle
     goal[0] = goal[1] = 90.; // position
     goal[2] = 0.; // orientation
     goal[3] = 0.; // joint angle
     goal[4] = 50.; // joint angle
     goal[5] = 40.; // joint angle
     goal[6] = 20.; // joint angle
     goal[7] = 50.; // joint angle
     goal[8] = 30.; // joint angle

     oc::ControlSpacePtr cmanifold(std::make_shared<oc::RealVectorControlSpace>(stateSpace, 8));

     // set the bounds for the control manifold
     ob::RealVectorBounds cbounds(8);
     // bounds for steering input
     cbounds.setLow(0, -1.);
     cbounds.setHigh(0, 1.);
     // bounds for speed input
     cbounds.setLow(1, 0.);
     cbounds.setHigh(1, 20.);
     // bounds for joint angular speed
     cbounds.setLow(2, -20.);
     cbounds.setHigh(2, 20.);
     cbounds.setLow(3, -20.);
     cbounds.setHigh(3, 20.);
     cbounds.setLow(4, -20.);
     cbounds.setHigh(4, 20.);
     cbounds.setLow(5, -20.);
     cbounds.setHigh(5, 20.);
     cbounds.setLow(6, -20.);
     cbounds.setHigh(6, 20.);
     cbounds.setLow(7, -20.);
     cbounds.setHigh(7, 20.);
     cmanifold->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

     oc::SimpleSetup setup(cmanifold);
     const oc::SpaceInformation *si = setup.getSpaceInformation().get();
     setup.setStartAndGoalStates(start, goal, 10.);
     setup.setStateValidityChecker([si](const ob::State *state)
         {
             return isStateValid(si, state);
         });
     setup.setStatePropagator([si](const ob::State *state, const oc::Control* control,
         const double duration, ob::State *result)
         {
             propagate(si, state, control, duration, result);
         });
     setup.getSpaceInformation()->setPropagationStepSize(.1);
     setup.getSpaceInformation()->setMinMaxControlDuration(1, 10);

     // try to solve the problem
     std::vector<double> path_1d;
     if (setup.solve(100))
     {
         // print the (approximate) solution path: print states along the path
         // and controls required to get from one state to the next
         oc::PathControl& path(setup.getSolutionPath());
         path.interpolate();

         // print out full state on solution path
         // (format: x, y, theta, v, u0, u1, dt)<< angles->values[1] << ' '
         for(unsigned int i=0; i<path.getStateCount(); ++i)
         {
             const ob::State* state = path.getState(i);
             const auto *se2 =
                 state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
             const auto *angles =
                 state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
             std::cout << se2->getX() << ' ' << se2->getY() << ' ' << se2->getYaw() << ' ' << angles->values[0] << ' ' << angles->values[1] << ' ' << angles->values[2] << ' ' << angles->values[3] << ' ' << angles->values[4] << ' ' << angles->values[5] << ' ';
             // store the joints to result
             path_1d.push_back(se2->getX());
	     path_1d.push_back(se2->getY());
	     path_1d.push_back(se2->getYaw());
             path_1d.push_back(angles->values[0]);
	     path_1d.push_back(angles->values[1]);
 	     path_1d.push_back(angles->values[2]);
	     path_1d.push_back(angles->values[3]);
	     path_1d.push_back(angles->values[4]);
	     path_1d.push_back(angles->values[5]);

             if (i==0)
                 // null controls applied for zero seconds to get to start state
                 std::cout << "0 0 0 0 0 0 0 0 0";
             else
             {
                 // print controls and control duration needed to get from state i-1 to state i
                 const double* u =
                     path.getControl(i-1)->as<oc::RealVectorControlSpace::ControlType>()->values;
                 std::cout << u[0] << ' ' << u[1] << ' ' << u[2] << ' ' << u[3] << ' ' << u[4] << ' ' << u[5] << ' ' << u[6] << ' ' << u[7] << ' ' << path.getControlDuration(i-1);
             }
             std::cout << std::endl;
         }
         if (!setup.haveExactSolutionPath())
         {
             std::cout << "Solution is approximate. Distance to actual goal is " <<
                 setup.getProblemDefinition()->getSolutionDifference() << std::endl;
         }
     }
     
     // the following is responsible in sending a service request to the visualization server
     ros::init(argc, argv, "joint_state_visualizer_node_client");

     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<ompl_test::ResultVisualizer>("result_visualizer");
     ompl_test::ResultVisualizer srv;
     srv.request.traj_1d = path_1d;
  
     if (client.call(srv))
     {
       ROS_INFO("Path sent to visualizer");
     }
     else
     {
       ROS_ERROR("Failed to call service result_visualizer");
       return 1;
     }

     return 0;
 }
