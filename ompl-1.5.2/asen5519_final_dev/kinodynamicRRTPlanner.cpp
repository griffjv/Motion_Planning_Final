#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/Obstacle.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <math.h>
#include <cmath>

/* Author: Griffin Van Anne */

namespace ob = ompl::base;
namespace oc = ompl::control;


// Methods - Implementation at bottom of code
std::vector<ob::Obstacle> BuildEnvironment(int num_obstacles);

bool CollisionCheck(ob::Obstacle obj, std::vector<ob::Obstacle> env);

void KinematicUAVODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot);

// TODO: Put in KinematicUAVPostIntegration if needed
// TODO: Put bounds checks on eta, rho, T inside state validity checker instead of ODE

bool isStateValid(const oc::SpaceInformation *si, const ob::State* state);

class UAVControlSpace : public oc::RealVectorControlSpace
{
public:
	UAVControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 3){
  }

}


void plan()
{
	std::ofstream PathResult("kinodynamic_result.txt");
	// construct the state space we are planning in - RE(3) in this case
	auto space(std::make_shared<ob::RealVectorStateSpace>());

	// add dimensions to state space
	space->addDimension("x", 0.0, 150.0);
	space->addDimension("y", 0.0, 150.0);
	space->addDimension("z", 0.0, 100.0);
	
	// TODO: ADD BOUNDS?
	space->addDimension("x_dot");
	space->addDimension("y_dot");
	space->addDimension("z_dot");
	const double g = 9.81; // m/s^2
	const double max_T = 2*g; //m/s^2
	const double max_eta = M_PI/4; //rad
	const double max_rho = M_PI/4; //rad
	const double max_T_dot = max_T/1; //one sec to reach max thrust
	const double max_eta_dot = max_eta/1; //one sec to reach peak eta displacement
	const double max_rho_dot = max_rho/1; //one sec to reach peak rho displacement
	space->addDimension("eta", -max_eta, max_eta);
	space->addDimension("rho", -max_rho, max_rho);
	space->addDimension("T", 0.0, max_T);	

	// space->setBounds(bounds);
	space->setup();
	
	// create control space
	auto cspace(std::make_shared<UAVControlSpace>(space));
	
	// set the bounds for control space
	ob::RealVectorBounds cbounds(3);
	cbounds.setLow(-max_eta_dot, 0); // eta
	cbounds.setHigh(max_eta_dot, 0);
	
	cbounds.setLow(-max_rho_dot, 0); // rho
	cbounds.setHigh(max_rho_dot, 0);
	
	cbounds.setLow(-max_T_dot, 0); // T
	cbounds.setHigh(max_T_dot, 0);
	
	// construct an instance of space information from this state space
	// SpaceInformation contains information about space such as, validity checker, distance calculator, state space, etc...
	auto si(std::make_shared<ob::SpaceInformation>(space));

	// set state validity checking for this space
	si->setStateValidityChecker(isStateValid);

	// create a start state
	ob::ScopedState<> start(space);
	start[0] = 140.0;
	start[1] = 140.0;
	start[2] = 0;
	start[3] = 0;
	start[4] = 0;
	start[5] = 0;
	start[6] = 0;
	start[7] = 0;
	start[8] = 0;

	// create a problem instance from space information
	auto pdef(std::make_shared<ob::ProblemDefinition>(si));

	// set the start and goal states
	//pdef->setStartAndGoalStates(start, goal);
	// set the start state
	pdef->clearStartStates();
	pdef->addStartState(start);
	
	// set the goal state
	pdef->clearGoal();
	auto gs(std::make_shared<ob::GoalState>(si));
	pdef->setGoal(std::make_shared<RigidBodyGoal>(si));
	
	// create a planner for the defined space
	auto planner(std::make_shared<oc::RRT>(si));

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
		// and inquire about the found path
		ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution:" << std::endl;

		// print the path to screen
		path->print(std::cout);
		path->print(PathResult);
	}
	else{
		std::cout << "No solution found" << std::endl;
		PathResult.close();
	}
}



// MAIN
int main(int /*argc*/, char** /*argv*/) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	
	srand((unsigned) time(NULL));
	plan();

	std::cout << std::endl << std::endl;



	return 0;
}




// ODE for kinematic UAV
void KinematicUAVODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
	const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
	
	// define system parameters
	const double g = 9.81; // m/s^2
	const double max_T = 2*g; // m/s^2
	const double max_eta = M_PI/4; // rad
	const double max_rho = M_PI/4; // rad
	
	// define state variables
	const double x = q[0]; const double y = q[1]; const double z = q[2];
	const double x_dot = q[3]; const double y_dot = q[4]; const double z_dot = q[5];
	const double eta = q[6]; const double rho = q[7]; const double T = q[8];
	
	// zero qdot
	qdot.resize(q.size(), 0);
	
	// state derivatives
	qdot[0] = x_dot;
	qdot[1] = y_dot;
	qdot[2] = z_dot;
	qdot[3] = -T * sin(eta) * cos(rho);
	qdot[4] = -T * sin(eta) * sin(rho);
	qdot[5] = T * cos(eta) - g;
	
	if(abs(eta) >= max_eta){ // keeping eta within bounds
		qdot[6] = 0; 
	}else{
		qdot[6] = u[0];
	}

	if(abs(rho) >= max_rho){ // keeping rho within bounds
		qdot[7] = 0; 
	}else{
		qdot[7] = u[1];
	}
	
	if(T >= max_T){ // keeping T within bounds
		qdot[8] = 0; 
	}else{
		qdot[8] = u[2];
	}
}

// check if state is valid
bool isStateValid(const oc::SpaceInformation *si, const ob::State* state) {

	// cast state to expected type, store as vector
	const auto* re3state = state->as<ob::RealVectorStateSpace::StateType>()->values;

	// update location of uav_obj based on state
	std::vector<float> location(re3state, re3state+3);
	uav_obj.set_values(location, 5, 5, 1); //size of uav is 5x5x1
	
	// collision check
	bool validity = !CollisionCheck(uav_obj, environment);

	return si->satisfiesBounds(state) && validity;
}


// Check for collisions assuming AABB representation of robot and obstacles
bool CollisionCheck(ob::Obstacle obj, std::vector<ob::Obstacle> env)
{

    // Get UAV min and max
    std::vector<float> uav_min = obj.aabb_min();
    std::vector<float> uav_max = obj.aabb_max();


    // Loop Through Array of Obstacles
    std::vector<float> obstacle_min, obstacle_max;
    int num_obstacles = env.size();
    for(int i = 0;i<num_obstacles;i++){

      // Get Obstacle min and max
      obstacle_min = env[i].aabb_min();
      obstacle_max = env[i].aabb_max();

      //Check if Box1's max is greater than Box2's min and Box1's min is less than Box2's max
      if(uav_max[0] > obstacle_min[0] &&
      uav_min[0] < obstacle_max[0] &&
      uav_max[1] > obstacle_min[1] &&
      uav_min[1] < obstacle_max[1] &&
      uav_max[2] > obstacle_min[2] &&
      uav_min[2] < obstacle_max[2])
      {return true;}
    }

    return false; //no collisions
}

std::vector<ob::Obstacle> BuildEnvironment(int num_obstacles){
  /*
  int max_h = 100;
  int min_h = 10;
  int max_w = 20;
  int max_d = 20;
  int min_w = 5;
  int min_d = 5;
  */

  float h_val[5] = {60, 34, 54.3, 100.0, 42.1};
  float w_val[5] = {68.2, 12.3, 14.5, 83, 73.2};
  float d_val[5] = {19.4, 12, 16.3, 85.3,43};


  float x_vals[5] = {1, 15, 30, 45, 20};
  float y_vals[5] = {1, 15, 30, 45, 60};
  float z_vals[5] = {0, 0, 0, 0, 0};

  std::vector<ob::Obstacle> env;
  for(int i=0;i<5;i++){
    std::vector<float> origin = {x_vals[i], y_vals[i], z_vals[i]};
    ob::Obstacle obs;
    obs.set_values(origin, w_val[i], d_val[i], h_val[i]);
    env.push_back(obs);
  }
  return env;
}




