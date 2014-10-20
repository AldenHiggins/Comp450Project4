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

// Check the environment for a point robot
bool pendulumStateValid(const control::SpaceInformation *si, const base::State *state)
{
    // Get the problem environment
    // std::vector<Rectangle> obstacles = problemEnvironment->getObstacles();

    // const base::RealVectorStateSpace::StateType *pos = state->as<base::RealVectorStateSpace::StateType>();
    // for (int obstacleIndex = 0; obstacleIndex < obstacles.size(); obstacleIndex++)
    // {
    //     Rectangle obstacle = obstacles[obstacleIndex];
    //     if (pos->values[0] > obstacle.bottomLeftCorner.x && pos->values[0] < obstacle.bottomRightCorner.x
    //      && pos->values[1] > obstacle.bottomLeftCorner.y && pos->values[1] < obstacle.topLeftCorner.y)
    //     {
    //         return false;
    //     }    
    // }

    return true;
}

// Definition of the ODE for the kinematic car.
// This method is analogous to the above KinematicCarModel::ode function.
void pendulumODE (const control::ODESolver::StateType& q, const control::Control* control, control::ODESolver::StateType& qdot)
{
    const double *u = control->as<control::RealVectorControlSpace::ControlType>()->values;

    if (q[0] > boost::math::constants::pi<double>()/2){
       qdot[0] = q[1]; 
    }
    else if (q[0] > 0){
        qdot[0] = q[1];
    }
    else if (q[0] > -1){
        qdot[0] = q[1];
    }

    // Zero out qdot
    qdot.resize (q.size (), 0);
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
    // ob::CompoundStateSpace::StateType& s = *result->as<ob::CompoundStateSpace::StateType>();
    // ob::SO2StateSpace::StateType& so2 = *s.as<ob::SO2StateSpace::StateType>(0);
    // ob::RealVectorStateSpace::StateType& angularVelocity = *s.as<ob::RealVectorStateSpace::StateType>(1);

    base::StateSpacePtr SO2(new base::SO2StateSpace());
    base::StateSpacePtr angularVelocity(new base::RealVectorStateSpace(1));
    base::StateSpacePtr stateSpace = SO2 + angularVelocity;

    // Bind angular velocity bounds
    base::RealVectorBounds velocityBound(1);
    // velocityBound.setLow(-3);
    // velocityBound.setHigh(3);
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
    // cbounds.setLow(-0.3);
    cbounds.setLow(-2);
    cbounds.setHigh(2);

    cmanifold->as<control::RealVectorControlSpace>()->setBounds(cbounds);

    // Set up control space
    control::SimpleSetup setup(cmanifold);

    // set state validity checking for this space
    setup.setStateValidityChecker(boost::bind(&pendulumStateValid, setup.getSpaceInformation().get(), _1));
    // Add ODE solver for the space
    control::ODESolverPtr odeSolver(new control::ODEBasicSolver<> (setup.getSpaceInformation(), &pendulumODE));
    setup.setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, &pendulumPostIntegration));
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

//     // Choose Planner
//     int choice;
//     do
//     {
//         std::cout << "Plan for: "<< std::endl;
//         std::cout << " (1) A point in 2D" << std::endl;
//         std::cout << " (2) A circle in 2D" << std::endl;
//         std::cout << " (3) A moving/rotating square in 2D" << std::endl;

//         std::cin >> choice;
//     } while (choice < 1 || choice > 3);
    plan(1);
}
