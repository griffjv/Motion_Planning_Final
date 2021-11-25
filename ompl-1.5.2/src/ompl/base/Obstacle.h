#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>

namespace ompl{
	namespace base{
		class Obstacle {
		    float width, depth, height;
		    std::vector<float> origin;
		public:
		  void set_values(std::vector<float> o, float w, float d, float h);
		  void move_origin(std::vector<float> state);
		  std::vector<float> aabb_min();
		  std::vector<float> aabb_max();
		};
	}	
}
#endif
