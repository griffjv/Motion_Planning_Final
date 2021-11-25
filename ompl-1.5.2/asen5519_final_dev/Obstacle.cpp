#include "Obstacle.h"
#include <vector>


std::vector<float> Obstacle::aabb_max() {
    std::vector<float> max;
    max.push_back(width + origin[0]); //x max value
    max.push_back(depth + origin[1]); //y max value
    max.push_back(height + origin[2]); //z max value

    return max;
}

std::vector<float> Obstacle::aabb_min(){
   return origin;
 }

void Obstacle::set_values(std::vector<float> o, float w, float d, float h) {
       width = w;
       depth = d;
       height = h;
       origin = o;
}

void Obstacle::move_origin(std::vector<float> state){
    origin = state;
}
