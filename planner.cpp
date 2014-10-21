/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include <ompl/base/spaces/SO2StateSpace.h>
#include <omplapp/config.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SimpleSetup.h>
#include <fstream>
#include <math.h>

#define CIRCLE_ROBOT_RADIUS .2
#define SQUARE_ROBOT_LENGTH .3

using namespace ompl;

// Check the environment for a pendulum
bool pendulumStateValid(const control::SpaceInformation *si, const base::State *state)
{
    return true;
}

// Definition of the ODE for the pendulum
void pendulumODE (const control::ODESolver::StateType& q, const control::Control* control, control::ODESolver::StateType& qdot)
{
    const double *u = control->as<control::RealVectorControlSpace::ControlType>()->values;

    // Zero out qdot
    qdot.resize (q.size (), 0);
    // Set qdot
    qdot[0] = q[1];
    qdot[1] = -9.81 * cos(q[0]) + u[0];
}

// This is a callback method invoked after numerical integration.
void pendulumPostIntegration (const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State *result)
{
    // // Normalize orientation between 0 and 2*pi
    base::SO2StateSpace SO2;
    SO2.enforceBounds (result->as<base::CompoundStateSpace::StateType>()->as<base::SO2StateSpace::StateType>(0));
}

void plan(int choice)
{
    // Define the state space
    base::StateSpacePtr SO2(new base::SO2StateSpace());
    base::StateSpacePtr angularVelocity(new base::RealVectorStateSpace(1));
    base::StateSpacePtr stateSpace = SO2 + angularVelocity;

    // Bind angular velocity bounds
    base::RealVectorBounds velocityBound(1);
    velocityBound.setLow(-300);
    velocityBound.setHigh(300);
    angularVelocity->as<base::RealVectorStateSpace>()->setBounds(velocityBound);

    // Define start and goal states
    base::ScopedState<> start(stateSpace);
    base::ScopedState<> goal(stateSpace);

    start[0] = -1 * boost::math::constants::pi<double>()/2;
    start[1] = 0;

    goal[0] = boost::math::constants::pi<double>()/2;
    goal[1] = 0;

    // Define control space
    control::ControlSpacePtr cmanifold(new control::RealVectorControlSpace(stateSpace, 1));

    // Set bounds of control space
    base::RealVectorBounds cbounds(1);
    cbounds.setLow(-2);
    cbounds.setHigh(2);
    cmanifold->as<control::RealVectorControlSpace>()->setBounds(cbounds);

    // Set up control space
    control::SimpleSetup setup(cmanifold);

    // Set state validity checking for this space
    setup.setStateValidityChecker(boost::bind(&pendulumStateValid, setup.getSpaceInformation().get(), _1));
    // Add ODE solver for the space
    control::ODESolverPtr odeSolver(new control::ODEBasicSolver<> (setup.getSpaceInformation(), &pendulumODE));
    // Add post integration function
    setup.setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, &pendulumPostIntegration));
    // Change planner variables
    setup.getSpaceInformation()->setPropagationStepSize(.1);
    setup.getSpaceInformation()->setMinMaxControlDuration(1, 3); // 2 3 default

    setup.setStartAndGoalStates(start, goal, 0.05);
    setup.setup();
    // Give the problem 30 seconds to solve
    if(setup.solve(30))
    {
        std::cout << "Found solution:" << std::endl;
        /// print the path to screen
        setup.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
    {
       std::cout << "No solution found" << std::endl; 
    }
}

int main(){
    // Choose Environment
//     int env;
//     do
//     {
//         std::cout << "Plan for: "<< std::endl;
//         std::cout << " (1) Maze Environment" << std::endl;
//         std::cout << " (2) Scattered Block Environment" << std::endl;

//         std::cin >> env;
//     } while (env < 1 || env > 2);
    plan(1);
}
