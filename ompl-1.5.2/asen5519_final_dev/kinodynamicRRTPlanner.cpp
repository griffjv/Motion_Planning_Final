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
#include <cctype>
#include <random>

using u32    = uint_least32_t; 
using engine = std::mt19937;




/* Author: Griffin Van Anne */

namespace ob = ompl::base;
namespace oc = ompl::control;

std::random_device os_seed;
const u32 seed = os_seed();
engine generator( seed );
std::uniform_int_distribution< u32 > distribute( 0, 6 );


// Methods - Implementation at bottom of code
std::vector<ob::Obstacle> BuildEnvironment();

bool CollisionCheck(ob::Obstacle obj, std::vector<ob::Obstacle> env);

void KinematicUAVODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot);

void KinematicUAVPostIntegration(const ob::State*, const oc::Control*, const double, ob::State *result);

/////// Global variables ///////
// build environment and uav object - use in state validity checker
std::vector<ob::Obstacle> environment = BuildEnvironment();
ob::Obstacle uav_obj;

// check if state is valid
bool isStateValid(const ob::State* state);

class UAVControlSpace : public oc::RealVectorControlSpace
{
public:
	UAVControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 3){
  }
};

class RigidBodyGoal : public ob::GoalSampleableRegion
{
public:
    RigidBodyGoal(const ob::SpaceInformationPtr &si) : ob::GoalSampleableRegion(si)
    {
    	threshold_ = 11.3; //sqrt(2) * 8 - radius to center of goal
    }
 
    virtual double distanceGoal(const ob::State *st) const override{
   	// goals
   	std::vector<float> goal_x, goal_y;
    	goal_x = {70, 140, 225, 25, 85, 20, 215};
    	goal_y = {30, 100, 85, 225, 150, 100, 160};
   	
   	// cast state to expected type, store as vector
	const auto* re3state = st->as<ob::RealVectorStateSpace::StateType>()->values;
	
	double distance = 0;

	double impact_speed = sqrt(2*9.81*re3state[2] + pow(re3state[5],2)); 
	
	if(impact_speed > 2.5){ //conservative estimate for max impact speed
		distance = distance + threshold_ + fabs(re3state[5]);
	}
		
	double lateral_speed = sqrt(re3state[3]*re3state[3] + re3state[4]*re3state[4]);
	
	if(lateral_speed > 5){
		distance = distance + threshold_ + lateral_speed;
	}
	
	double dist1 = 1000;
	for(int i = 0; i<7;i++){
	// Check if x,y has reached goal position
		double dx1 = fabs(re3state[0] - goal_x[i]);
		double dy1 = fabs(re3state[1] - goal_y[i]);
		
		if(sqrt(dx1*dx1 + dy1*dy1 + re3state[2]*re3state[2]) < dist1){
			dist1 = sqrt(dx1*dx1 + dy1*dy1 + re3state[2]*re3state[2]);
		}
	}

	distance = dist1+distance;	
	return distance;
    }
    
    void sampleGoal(ob::State *st) const override{
    	std::vector<float> goal_x, goal_y;
    	goal_x = {70, 140, 225, 25, 85, 20, 215};
    	goal_y = {30, 100, 85, 225, 150, 100, 160};
    	
	int num = distribute( generator );

	st->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_x[num];
	st->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_y[num];
	st->as<ob::RealVectorStateSpace::StateType>()->values[2] =0;
	st->as<ob::RealVectorStateSpace::StateType>()->values[8] =9.81; //hover thresh - should be maybe >9.81, otherwise it is still accel down
	
	// TODO: Set random goal for the other states, or just default to 0?
	st->as<ob::RealVectorStateSpace::StateType>()->values[3] =0;
	st->as<ob::RealVectorStateSpace::StateType>()->values[4] =0;
	st->as<ob::RealVectorStateSpace::StateType>()->values[5] =0;
	st->as<ob::RealVectorStateSpace::StateType>()->values[6] =0;
	st->as<ob::RealVectorStateSpace::StateType>()->values[7] =0;
    }
    
    virtual unsigned int maxSampleCount() const override{
    	return 100000;
    }
};


double plan(double control_scale)
{
	std::ofstream PathResult("kino_result_goal7.txt");
	// construct the state space we are planning in - RE(3) in this case
	auto space(std::make_shared<ob::RealVectorStateSpace>());

	// add dimensions to state space
	space->addDimension("x", 0.0, 250.0);
	space->addDimension("y", 0.0, 250.0);
	space->addDimension("z", 0.0, 200.0);
	
	space->addDimension("x_dot", -15, 15);
	space->addDimension("y_dot", -15, 15);
	space->addDimension("z_dot", -15, 15);
	const double g = 9.81; // m/s^2
	const double max_T = 2*g; //m/s^2
	const double max_eta = M_PI/8; //rad
	const double max_rho = M_PI/2; //rad
	const double max_T_dot = max_T/1 * .75; //one sec to reach max thrust
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
	cbounds.setLow(0, -max_eta_dot); // eta
	cbounds.setHigh(0, max_eta_dot);
	
	cbounds.setLow(1, -max_rho_dot); // rho
	cbounds.setHigh(1, max_rho_dot);
	
	cbounds.setLow(2, -max_T_dot); // T
	cbounds.setHigh(2, max_T_dot);
	
	cspace->setBounds(cbounds);
	// construct an instance of space information from this state space
	// SpaceInformation contains information about space such as, validity checker, distance calculator, state space, etc...
	auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

	// set state validity checking for this space
	si->setStateValidityChecker(isStateValid);

	// create a start state
	ob::ScopedState<> start(space);
	start[0] = 240;
	start[1] = 240;
	start[2] = 0;
	start[3] = 0;
	start[4] = 0;
	start[5] = 0;
	start[6] = 0;
	start[7] = 0;
	start[8] = 9.81; //thrust

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
	auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &KinematicUAVODE));
	si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicUAVPostIntegration));
	si->setPropagationStepSize(0.2);
	si->setMaxControlDuration(10);
	si->setMinControlDuration(1);
	
	//si->setup();
	
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
	ob::PlannerStatus solved = planner->ob::Planner::solve(100.0);

	if (solved)
	{
		// and inquire about the found path
		ob::PathPtr path = pdef->getSolutionPath();;
		std::cout << "Found solution:" << std::endl;
		
		// print the path to screen
		path->as<oc::PathControl>()->printAsMatrix(std::cout);
		std::cout<<pdef->getSolutionDifference()<<std::endl;
		
		// save path to file
		path->as<oc::PathControl>()->printAsMatrix(PathResult);
	}
	else{
		std::cout << "No solution found" << std::endl;
		PathResult.close();
	}
	planner->clear();
	return pdef->getSolutionDifference();
}



// MAIN
int main(int /*argc*/, char** /*argv*/) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	
	// benchmark results
	double result;
	std::vector<float>  distances =  {0,0,0,0,0};
	std::vector<double> times = {.05, .1, .2, .3, .5};
	int tally = 0;
	for(int i = 0; i<8; i++){
		if(plan(.1) == 0){
			tally++;
		}
	}
	std::cout<<"Tally: "<<tally<<std::endl;
	/*
	for(int j = 0; j<1; j++){
		result = 0;
		for(int i = 0; i<30; i++){
			// plan
			result = result + plan(times[j]);
			std::cout<<i<<std::endl;
		}
		distances[j] = result/30;
		std::cout<<distances[j]<<std::endl;
	}
	
	std::cout<<"Results: "<<std::endl;
	for(int j = 0; j<1; j++){
		std::cout<<distances[j]<<std::endl;
	}
	*/
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
	const double max_eta = M_PI/8; // rad
	const double max_rho = M_PI/2; // rad
	
	// define state variables
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
	
	
	// KEEP CONTROLS WITHIN BOUNDS
	if(eta >= max_eta && u[0]>0){
		qdot[6] = 0; 
	}else if(eta <= -max_eta && u[0]<0){
		qdot[6] = 0;
	}else{
		qdot[6] = u[0];
	}

	
	if(rho >= max_rho && u[1]>0){
		qdot[7] = 0; 
	}else if(rho <= -max_rho && u[1]<0){
		qdot[7] = 0;
	}else{
		qdot[7] = u[1];
	}
	
	
	if(T >= max_T && u[2]>0){
		qdot[8] = 0; 
	}else if(T <= 0 && u[2]<0){
		qdot[8] = 0;
	}else{
		qdot[8] = u[2];
	}
	
	if(fabs(q[2])<.1){ // on the ground
		if(fabs(z_dot) > 1){ // set all state derivatives to 0
			qdot[0] = 0; qdot[1] = 0; qdot[2] = 0;
			qdot[3] = 0; qdot[4] = 0; qdot[5] = 0;
		}
	}
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

std::vector<ob::Obstacle> BuildEnvironment(){
  /*
  int max_h = 150;
  int min_h = 10;
  int max_w = 20;
  int max_d = 20;
  int min_w = 5;
  int min_d = 5;
  */

  float h_val[15] = {60, 34, 54.3, 150.0, 42.1, 69, 30, 87, 94, 60, 128, 59, 142, 72, 49};
  float w_val[15] = {68.2, 20, 24, 83, 73.2, 31, 49, 47, 59, 31, 35, 57, 48, 41, 21};
  float d_val[15] = {19.4, 20, 16.3, 85.3,43, 59, 29, 27, 50, 70, 55, 53, 75, 37, 25};


  float x_vals[15] = {1, 15, 30, 45, 170, 10, 200, 180, 120, 43, 86, 119, 152, 164, 125};
  float y_vals[15] = {1, 15, 30, 45, 20, 145, 200, 105, 120, 93, 23, 25, 44, 143, 210};
  float z_vals[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  std::vector<ob::Obstacle> env;
  for(int i=0;i<15;i++){
    std::vector<float> origin = {x_vals[i], y_vals[i], z_vals[i]};
    ob::Obstacle obs;
    obs.set_values(origin, w_val[i], d_val[i], h_val[i]);
    env.push_back(obs);
  }
  return env;
}

// check if state is valid
bool isStateValid(const ob::State* state) {

	// cast state to expected type, store as vector
	const auto* re3state = state->as<ob::RealVectorStateSpace::StateType>()->values;

	// update location of uav_obj based on state
	std::vector<float> location(re3state, re3state+3);
	uav_obj.set_values(location, 2, 2, .5); //size of uav is 2x2x1
	
	// collision check
	bool validity = !CollisionCheck(uav_obj, environment);

	return validity;
}

void KinematicUAVPostIntegration(const ob::State*, const oc::Control*, const double, ob::State *result){
	
		// cast state to expected type, store as vector
	auto* re3state = result->as<ob::RealVectorStateSpace::StateType>()->values;

	if(re3state[2] < 0){
		re3state[2] = 0;
	}
	
	/* normalize rho to 0-2*pi
	if(re3state[7] > 2*M_PI){
		re3state[7] = re3state[7] - 2*M_PI;
	}else if(re3state[7] < 0){
		re3state[7] = re3state[7] + 2*M_PI;
	}
	*/
}	

