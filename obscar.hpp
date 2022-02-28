#ifndef __OBS_CAR_STRUCT_H
#define __OBS_CAR_STRUCT_H

#include <cmath>
#include <vector>

using namespace std;

namespace riskfield {

class ObsCar {
    public:

        ObsCar(vector<float> dimension = {0,0,0}, vector<float> location = {0,0,0}, float y = 0, float v = 0, float a = 0)
                : l(dimension[0]), w(dimension[1]), h(dimension[2]),
                x(location[0]), y(location[1]), z(location[2]),
                yaw(y), velocity(v), acc(a) {}         // 构造函数

        ~ObsCar() {}

        float get_x() const { return x; }

        float get_y() const { return y; }

        float get_z() const { return z; }

        vector<float> get_dimension() const {
            vector<float> dimension = {l, w, h};
            return dimension;
        }

        float get_yaw() const { return yaw; }

        float get_v() const { return velocity; }

        float get_a() const { return acc; }


    private:
        ObsCar(const ObsCar&) = delete;              // copy cotr 明确拒绝
        ObsCar& operator=(const ObsCar&) = delete;   // 明确拒绝 

        float l, w, h;  // long, width, height
        float x, y, z;  // location
        float yaw; // angle
        float velocity;
        float acc;

};

};  // riskfield

#endif  // __OBS_CAR_STRUCT_H
