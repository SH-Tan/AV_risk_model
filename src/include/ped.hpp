#ifndef __OBS_PED_STRUCT_H
#define __OBS_PED_STRUCT_H

#include <cmath>
#include <vector>
#include <string>

using namespace std;

namespace riskfield {

class ObsPed {
    public:

        ObsPed(vector<float> &location, string type_ = "")
                : x(location[0]), y(location[1]), z(location[2]),
                  type(type_) {}         // 构造函数

        ~ObsPed() {}

        float get_x() const { return x; }

        float get_y() const { return y; }

        float get_z() const { return z; }

        string get_type() const { return type; }

        // float get_yaw() const { return yaw; }

        // float get_a() const { return acc; }


    private:
        ObsPed(const ObsPed&) = delete;              // copy cotr 明确拒绝
        ObsPed& operator=(const ObsPed&) = delete;   // 明确拒绝 

        float x, y, z;  // location
        
        string type;

};

}  // riskfield

#endif  // __OBS_PED_STRUCT_H
