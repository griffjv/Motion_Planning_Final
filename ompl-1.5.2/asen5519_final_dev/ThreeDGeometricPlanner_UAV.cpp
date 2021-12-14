#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Obstacle.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <string>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <cctype>
#include <random>

using u32    = uint_least32_t; 
using engine = std::mt19937;

namespace ob = ompl::base;
namespace og = ompl::geometric;

std::random_device os_seed;
const u32 seed = os_seed();
engine generator( seed );
std::uniform_int_distribution< u32 > distribute( 0, 6 );

/* Author: Griffin Van Anne */
std::vector<ob::Obstacle> BuildEnvironment();

bool CollisionCheck(ob::Obstacle obj, std::vector<ob::Obstacle> env);



/////// Global variables ///////
// build environment and uav object - use in state validity checker
std::vector<ob::Obstacle> environment = BuildEnvironment();
ob::Obstacle uav_obj;


class RigidBodyGoal : public ob::GoalSampleableRegion
{
public:
    RigidBodyGoal(const ob::SpaceInformationPtr &si) : ob::GoalSampleableRegion(si)
    {
    	threshold_ = 1; //not sure what this is yet, rad of goal?
    }
 
    virtual double distanceGoal(const ob::State *st) const override{
	std::vector<float> goal_x, goal_y;
    	goal_x = {70, 140, 225, 25, 85, 20, 215};
    	goal_y = {30, 100, 85, 225, 150, 100, 160};
    	
   	// cast state to expected type, store as vector
	const auto* re3state = st->as<ob::RealVectorStateSpace::StateType>()->values;
	
	double dist2 = 0;
	if (re3state[2] > 0.1){ //must be approx on ground
		dist2 = re3state[2]+threshold_;
	}
	
	double dist1 = 1000;
	for(int i = 0; i<7;i++){
		// Check if x,y has reached goal position
		double dx1 = fabs(re3state[0] - goal_x[6]);
		double dy1 = fabs(re3state[1] - goal_y[6]);
		
		if(sqrt(dx1*dx1 + dy1*dy1 + re3state[2]*re3state[2]) < dist1){
			dist1 = sqrt(dx1*dx1 + dy1*dy1 + re3state[2]*re3state[2]);
		}
	}
	return dist1+dist2;	
    }
    
    void sampleGoal(ob::State *st) const override{
    	std::vector<float> goal_x, goal_y;
    	goal_x = {70, 140, 225, 25, 85, 20, 215};
    	goal_y = {30, 100, 85, 225, 150, 100, 160};

	int num = distribute( generator );
	num = 6;
	st->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_x[num];
	st->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_y[num];
	st->as<ob::RealVectorStateSpace::StateType>()->values[2] =0;			
    }
    
    virtual unsigned int maxSampleCount() const override{
    	return 100000;
    }
};


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


void plan()
{
	std::ofstream PathResult("geo_result_goal7.txt");
	// construct the state space we are planning in - RE(3) in this case
	auto space(std::make_shared<ob::RealVectorStateSpace>());

	// add dimensions to state space
	space->addDimension("x", 0.0, 250.0);
	space->addDimension("y", 0.0, 250.0);
	space->addDimension("z", 0.0, 200.0);

	//space->setBounds(bounds);
	space->setup();

	// construct an instance of space information from this state space
	// SpaceInformation contains information about space such as, validity checker, distance calculator, state space, etc...
	auto si(std::make_shared<ob::SpaceInformation>(space));

	// set state validity checking for this space
	si->setStateValidityChecker(isStateValid);

	// create a start state
	ob::ScopedState<> start(space);
	start[0] = 240;
	start[1] = 240;
	start[2] = 0;

	// create a goal state
	ob::ScopedState<> goal(space);

	// create a problem instance from space information
	auto pdef(std::make_shared<ob::ProblemDefinition>(si));

	// set the start state
	pdef->clearStartStates();
	pdef->addStartState(start);
	
	// set the goal state
	pdef->clearGoal();
	auto gs(std::make_shared<ob::GoalState>(si));
	pdef->setGoal(std::make_shared<RigidBodyGoal>(si));
	
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

int main(int /*argc*/, char** /*argv*/) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	
	plan();

	std::cout << std::endl << std::endl;



	return 0;
}

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
