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

 /* Author: Elizabeth Fudge */

 #include <ompl/base/goals/GoalState.h>
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/base/spaces/DiscreteStateSpace.h>
 #include <ompl/control/spaces/RealVectorControlSpace.h>
 #include <ompl/control/SimpleSetup.h>
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
     ob::RealVectorStateSpace::StateType& velocity = *s.as<ob::RealVectorStateSpace::StateType>(1);

     si->getStateSpace()->copyState(result, state);
     for(int i=0; i<nsteps; i++)
     {
         velocity.values[0] = velocity.values[0] + dt*u[1];
         // propagate the joint angle
         velocity.values[1] = velocity.values[1] + dt*u[2];
         se2.setX(se2.getX() + dt * velocity.values[0] * cos(se2.getYaw()));
         se2.setY(se2.getY() + dt * velocity.values[0] * sin(se2.getYaw()));
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


 int main(int /*argc*/, char** /*argv*/)
 {
     // plan for hybrid car in SE(2) with discrete gears
     auto SE2(std::make_shared<ob::SE2StateSpace>());
     auto velocity(std::make_shared<ob::RealVectorStateSpace>(2));
     ob::StateSpacePtr stateSpace = SE2 + velocity;

     // set the bounds for the R^2 part of SE(2)
     ob::RealVectorBounds bounds(2);
     bounds.setLow(-100);
     bounds.setHigh(100);
     SE2->setBounds(bounds);

     // set the bounds for the velocity
     ob::RealVectorBounds velocityBound(2);
     velocityBound.setLow(0, 0);
     velocityBound.setHigh(0, 60);
     // set the bounds for the joint angle
     velocityBound.setLow(1, 0);
     velocityBound.setHigh(1, 60);
     velocity->setBounds(velocityBound);

     // create start and goal states
     ob::ScopedState<> start(stateSpace);
     ob::ScopedState<> goal(stateSpace);

     // Both start and goal are states with high velocity with the car in third gear.
     // However, to make the turn, the car cannot stay in third gear and will have to
     // shift to first gear.
     start[0] = start[1] = -90.; // position
     start[2] = boost::math::constants::pi<double>()/2; // orientation
     start[3] = 40.; // velocity
     start[4] = 0.; // joint angle
     goal[0] = goal[1] = 90.; // position
     goal[2] = 0.; // orientation
     goal[3] = 40.; // velocity
     goal[4] = 50.; // joint angle

     oc::ControlSpacePtr cmanifold(std::make_shared<oc::RealVectorControlSpace>(stateSpace, 3));

     // set the bounds for the control manifold
     ob::RealVectorBounds cbounds(3);
     // bounds for steering input
     cbounds.setLow(0, -1.);
     cbounds.setHigh(0, 1.);
     // bounds for brake/gas input
     cbounds.setLow(1, -20.);
     cbounds.setHigh(1, 20.);
     // bounds for joint angular speed
     cbounds.setLow(2, -20);
     cbounds.setHigh(2, 20);
     cmanifold->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

     oc::SimpleSetup setup(cmanifold);
     const oc::SpaceInformation *si = setup.getSpaceInformation().get();
     setup.setStartAndGoalStates(start, goal, 5.);
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
     setup.getSpaceInformation()->setMinMaxControlDuration(2, 3);

     // try to solve the problem
     if (setup.solve(30))
     {
         // print the (approximate) solution path: print states along the path
         // and controls required to get from one state to the next
         oc::PathControl& path(setup.getSolutionPath());

         // print out full state on solution path
         // (format: x, y, theta, v, u0, u1, dt)
         for(unsigned int i=0; i<path.getStateCount(); ++i)
         {
             const ob::State* state = path.getState(i);
             const auto *se2 =
                 state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
             const auto *velocity =
                 state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
             std::cout << se2->getX() << ' ' << se2->getY() << ' ' << se2->getYaw() << ' ' << velocity->values[0] << ' ' << velocity->values[1]<< ' ';
             if (i==0)
                 // null controls applied for zero seconds to get to start state
                 std::cout << "0 0 0 0";
             else
             {
                 // print controls and control duration needed to get from state i-1 to state i
                 const double* u =
                     path.getControl(i-1)->as<oc::RealVectorControlSpace::ControlType>()->values;
                 std::cout << u[0] << ' ' << u[1] << ' ' << u[2] << ' ' << path.getControlDuration(i-1);
             }
             std::cout << std::endl;
         }
         if (!setup.haveExactSolutionPath())
         {
             std::cout << "Solution is approximate. Distance to actual goal is " <<
                 setup.getProblemDefinition()->getSolutionDifference() << std::endl;
         }
     }

     return 0;
 }
