#ifndef __EGO_CAR_STRUCT_H
#define __EGO_CAR_STRUCT_H

#include <iostream>
#include <vector>

using namespace std;

namespace riskfield {

class EgoCar {
    public:
        static EgoCar* create_ego(vector<float> dimension, vector<float> location, float y, float v, float a) {
            m_ego = new EgoCar(dimension, location, y, v, a);
            return m_ego;
        };

        static EgoCar* get_ego () {
            if (m_ego != nullptr) {
                return m_ego;
            }
            return nullptr;
        };

    private:
        EgoCar(vector<float> dimension = {0,0,0}, vector<float> location = {0,0,0}, float y = 0, float v = 0, float a = 0)
            : l(dimension[0]), w(dimension[1]), h(dimension[2]),
              x(location[0]), y(location[1]), z(location[2]),
              yaw(y), velocity(v), acc(a) {}                                  // 构造函数私有
        EgoCar(const EgoCar&) = delete;             // copy cotr 明确拒绝
        EgoCar& operator=(const EgoCar&) = delete;  // 明确拒绝 

        static EgoCar* m_ego;

        ~EgoCar() {
            if (m_ego != nullptr) {
                delete m_ego;
            }
        }

        float l, w, h;  // long, width, height
        float x, y, z;  // location
        float yaw; // angle
        float velocity;
        float acc;

};

};  // riskfield

#endif  // __EGO_CAR_STRUCT_H
