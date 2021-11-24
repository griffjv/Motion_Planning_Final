#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>

using namespace std;

class Obstacle {
    float width, depth, height;
    std::vector<float> origin;
public:
  void set_values(std::vector<float> o, float w, float d, float h) {
      width = w;
      depth = d;
      height = h;
      origin = o;
    }
    void move_origin(std::vector<float> state){
      origin = state;
    }
    std::vector<float> aabb_min() { return origin; }
    std::vector<float> aabb_max() {
        std::vector<float> max;
        max.push_back(width + origin[0]); //x max value
        max.push_back(depth + origin[1]); //y max value
        max.push_back(height + origin[2]); //z max value

        return max;
    }
};



// TODO: first input =  uav_obstacle, second input = array of obstacles?
// TODO: does aabb_max need to return anything, or is the array passed by refrence?... think the latter, if so, adjust

bool CollisionCheck(Obstacle uav_obj, std::vector<Obstacle> env)
{

    // Get UAV min and max
    std::vector<float> uav_min = uav_obj.aabb_min();
    std::vector<float> uav_max = uav_obj.aabb_max();


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




std::vector<Obstacle> BuildEnvironment(int num_obstacles){
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

  std::vector<Obstacle> env;
  for(int i=0;i<5;i++){
    std::vector<float> origin = {x_vals[i], y_vals[i], z_vals[i]};
    Obstacle obs;
    obs.set_values(origin, w_val[i], d_val[i], h_val[i]);
    env.push_back(obs);
  }
  return env;
}

int main(){
 std::vector<Obstacle> environment = BuildEnvironment(5);
 Obstacle uav_obj;
 std::vector<float> state = {1, 1, 50}; 
 uav_obj.set_values(state, 5, 5, 1);

 bool collision = CollisionCheck(uav_obj, environment);
 if(collision){
    cout<<"Collision has occured"<<endl;
 }else{
     cout<<"No Collision has occured"<<endl;
 }
}
