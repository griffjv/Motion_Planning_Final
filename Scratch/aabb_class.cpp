#include <iostream>
using namespace std;

class Obstacle {
    float width, depth, height;
    float* origin;
public:
    void set_values(float*, float, float, float);
    float* aabb_min() { return origin; }
    float* aabb_max(float* max) {
        cout << origin << endl;
        cout << origin[0] << endl;
        max[0] = width + origin[0]; //x max value
        max[1] = depth + origin[1]; //y max value
        max[2] = height + origin[2]; //z max value

        return max;
    }
};

void Obstacle::set_values(float o[], float w, float d, float h) {
    width = w;
    depth = d;
    height = h;
    origin = o;
}

int main() {
    Obstacle obst;
    float org1[3] = { 1, 1, 1 };
    obst.set_values(org1, 2, 2, 4);
    float maxval[3];
    cout << obst.aabb_max(&maxval[0])[2] << endl;
    return 0;
}