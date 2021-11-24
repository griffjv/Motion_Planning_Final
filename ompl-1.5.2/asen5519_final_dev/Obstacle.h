#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>

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
}
