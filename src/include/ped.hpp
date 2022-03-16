#ifndef __OBS_PED_STRUCT_H
#define __OBS_PED_STRUCT_H

#include <cmath>
#include <vector>
#include <string>

using namespace std;

namespace riskfield {

class ObsPed {
    public:

        ObsPed(vector<float> &dimension, vector<float> &location, float &v, string type_ = "")
                : l(dimension[0]), w(dimension[1]), h(dimension[2]),
                x(location[0]), y(location[1]), z(location[2]),
                velocity(v), type(type_) {}         // 构造函数

        ~ObsPed() {}

        float get_x() const { return x; }

        float get_y() const { return y; }

        float get_z() const { return z; }

        vector<float> get_dimension() const {
            vector<float> dimension = {l, w, h};
            return dimension;
        }

        // float get_yaw() const { return yaw; }

        float get_v() const { return velocity; }

        // float get_a() const { return acc; }


    private:
        ObsPed(const ObsPed&) = delete;              // copy cotr 明确拒绝
        ObsPed& operator=(const ObsPed&) = delete;   // 明确拒绝 

        float l, w, h;  // long, width, height
        float x, y, z;  // location
        // float yaw; // angle
        float velocity;
        // float acc;
        string type;

};

}  // riskfield

#endif  // __OBS_PED_STRUCT_H
