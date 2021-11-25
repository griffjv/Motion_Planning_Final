#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/* Author: Griffin Van Anne */
/* Source: Ioan Sucan */

bool isStateValid(const ob::State* state) {

	// cast state to expected type
	std::cout << state << std::endl;
	const auto* re3state = state->as<ob::RealVectorStateSpace::StateType>();
	std::cout << re3state << std::endl;

	// TODO :: update checker to take in multiple objects and check more complex geometric collisions
	// Testing Rectangle spanning 4.5<x<5.5, 0<y<10, 0<z<5
	// check validity of state defined by re3state
	//bool validity = (4.5 <= re3state[0] && re3state[0] <= 5.5) && (0 <= re3state[1] && re3state[1] <= 10) && (0 <= re3state[2] && re3state[2] <= 5);

	bool validity = false;
	return validity;
}

void plan()
{
	// construct the state space we are planning in - RE(3) in this case
	auto space(std::make_shared<ob::RealVectorStateSpace>());

	ob::RealVectorBounds bounds(3);
	// uniform bounds for all dimensions
	bounds.setLow(0);
	bounds.setHigh(10);

	space->setBounds(bounds);

	// construct an instance of space information from this state space
	// SpaceInformation contains information about space such as, validity checker, distance calculator, state space, etc...
	auto si(std::make_shared<ob::SpaceInformation>(space));

	// set state validity checking for this space
	si->setStateValidityChecker(isStateValid);

	// create a start state
	ob::ScopedState<> start(space);
	start[0] = 2;
	start[1] = 2;
	start[2] = 0;

	// create a goal state
	ob::ScopedState<> goal(space);
	goal[0] = 8;
	goal[1] = 8;
	goal[2] = 0;

	// create a problem instance from space information
	auto pdef(std::make_shared<ob::ProblemDefinition>(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

	// create a planner for the defined space
	auto planner(std::make_shared<og::RRT>(si));

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	// print the settings for this space
	si->printSettings(std::cout);

	// print the problem settings
	pdef->print(std::cout);

	// attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

	if (solved)
	{
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution:" << std::endl;

		// print the path to screen
		path->print(std::cout);
	}
	else
		std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char** /*argv*/) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	plan();

	std::cout << std::endl << std::endl;

	return 0;
}
