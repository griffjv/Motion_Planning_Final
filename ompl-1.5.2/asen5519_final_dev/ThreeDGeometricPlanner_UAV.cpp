#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Obstacle.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalSampleableRegion.h>


#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;





/* Author: Griffin Van Anne */
std::vector<ob::Obstacle> BuildEnvironment(int num_obstacles);

bool CollisionCheck(ob::Obstacle obj, std::vector<ob::Obstacle> env);



/////// Global variables ///////
// build environment and uav object - use in state validity checker
std::vector<ob::Obstacle> environment = BuildEnvironment(5);
ob::Obstacle uav_obj;


class RigidBodyGoal : public ob::GoalSampleableRegion
{
public:
    RigidBodyGoal(const ob::SpaceInformationPtr &si) : ob::GoalSampleableRegion(si)
    {
    	threshold_ = 1; //not sure what this is yet, rad of goal?
    }
 
    virtual double distanceGoal(const ob::State *st) const override{
   	// cast state to expected type, store as vector
	const auto* re3state = st->as<ob::RealVectorStateSpace::StateType>()->values;
	
	if (re3state[2] > 0.1){ //must be approx on ground
		return re3state[2]+threshold_;
	}else{
		double dx1 = fabs(re3state[0] - 23);
		double dy1 = fabs(re3state[1] - 33);
		
		double dx2 = fabs(re3state[0] - 43);
		double dy2 = fabs(re3state[1] - 24);
		
		double dist1 = sqrt(dx1*dx1 + dy1*dy1 + re3state[2]*re3state[2]);
		double dist2 = sqrt(dx2*dx2 + dy2*dy2 + re3state[2]*re3state[2]);
		
		if(dist1 > dist2){
			return dist2;
		}else{
			return dist1;
		}	
	}
    }
    
    void sampleGoal(ob::State *st) const override{
	int num = rand() % 2;
	
	if(num == 0){
		st->as<ob::RealVectorStateSpace::StateType>()->values[0]=23;
		st->as<ob::RealVectorStateSpace::StateType>()->values[1] =33;
		st->as<ob::RealVectorStateSpace::StateType>()->values[2] =0;
		std::cout<<"goal1"<<std::endl;
	}else{
		st->as<ob::RealVectorStateSpace::StateType>()->values[0]=43;
		st->as<ob::RealVectorStateSpace::StateType>()->values[1] =24;
		st->as<ob::RealVectorStateSpace::StateType>()->values[2] =0;
		std::cout<<"goal2"<<std::endl;
	}
	// cast state to expected type, store as vector
	auto* re3state = st->as<ob::RealVectorStateSpace::StateType>()->values;
	std::cout<<re3state[0]<<std::endl;
	
    }
    
    virtual unsigned int maxSampleCount() const override{
    	return 100;
    }
};


bool isStateValid(const ob::State* state) {

	// cast state to expected type, store as vector
	const auto* re3state = state->as<ob::RealVectorStateSpace::StateType>()->values;

	// update location of uav_obj based on state
	std::vector<float> location(re3state, re3state+3);
	uav_obj.set_values(location, 5, 5, 1); //size of uav is 5x5x1
	
	// collision check
	bool validity = !CollisionCheck(uav_obj, environment);

	return validity;
}


void plan()
{



	std::ofstream PathResult("path_result.txt");
	// construct the state space we are planning in - RE(3) in this case
	auto space(std::make_shared<ob::RealVectorStateSpace>());

	// add dimensions to state space
	space->addDimension("x", 0.0, 150.0);
	space->addDimension("y", 0.0, 150.0);
	space->addDimension("z", 0.0, 100.0);

	//ob::RealVectorBounds bounds(3);
	// uniform bounds for all dimensions
	//bounds.setLow(0);
	//bounds.setHigh(10);

	//space->setBounds(bounds);
	space->setup();

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

	// create a goal state
	ob::ScopedState<> goal(space);
	goal[0] = 25.0;
	goal[1] = 70.0;
	goal[2] = 42.1;

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
	
	srand((unsigned) time(NULL));
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
