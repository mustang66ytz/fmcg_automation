#include <ompl/control/SpaceInformation.h>
 #include <ompl/base/goals/GoalState.h>
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/control/spaces/RealVectorControlSpace.h>
 #include <ompl/control/planners/kpiece/KPIECE1.h>
 #include <ompl/control/planners/rrt/RRT.h>
 #include <ompl/control/planners/est/EST.h>
 #include <ompl/control/planners/syclop/SyclopRRT.h>
 #include <ompl/control/planners/syclop/SyclopEST.h>
 #include <ompl/control/planners/pdst/PDST.h>
 #include <ompl/control/planners/syclop/GridDecomposition.h>
 #include <ompl/control/SimpleSetup.h>
 #include <ompl/config.h>
 #include <iostream>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
 {
     const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
     const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
     const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
     const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

     result->as<ob::SE2StateSpace::StateType>()->setXY(
         pos[0] + ctrl[0] * duration * cos(rot),
         pos[1] + ctrl[0] * duration * sin(rot));
     result->as<ob::SE2StateSpace::StateType>()->setYaw(
         rot    + ctrl[1] * duration);
 }

int main(int /*argc*/, char ** /*argv*/)
 {
     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

     // plan();
     //
     // std::cout << std::endl << std::endl;
     //
     //planWithSimpleSetup();

     return 0;
 }
