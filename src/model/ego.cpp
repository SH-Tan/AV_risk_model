#include "ego.h"

namespace riskfield {

    atomic<EgoCar*> EgoCar::m_ego{nullptr};
    EgoCar::Deletor EgoCar::deletor;

    EgoCar::EgoCar(vector<float> dimension, vector<float> location, float y, float v, float a)
            : l(dimension[0]), w(dimension[1]), h(dimension[2]),
              x(location[0]), y(location[1]), z(location[2]),
              yaw(y), velocity(v), acc(a) {}         // 构造函数私有

    void EgoCar::refresh_ego(vector<float> dimension, vector<float> location, float y, float v, float a) {

            EgoCar* car = m_ego;
            lock_guard<mutex> lock(mtx); 

            if (m_ego == nullptr) {
                // double check
                if ((car = m_ego) == nullptr) {
                    m_ego = car = new EgoCar(dimension, location, y, v, a);
                    // return car;
                }
            } 
            update_ego(car, dimension, location, y, v, a);
            m_ego = car;
            // return car;
        }

        EgoCar* EgoCar::get_car() {
            return m_ego;
        }

        void EgoCar::update_ego (EgoCar* &car, vector<float> dimension, vector<float> location, float y, float v, float a) {
            car->l = dimension[0];
            car->w = dimension[1];
            car->h = dimension[2];
            car->x = location[0];
            car->y = location[1];
            car->z = location[2];
            car->yaw = y;
            car->velocity = v;
            car->acc = a;
        }


        float EgoCar::get_x() const { return x; }

        float EgoCar::get_y() const { return y; }

        float EgoCar::get_z() const { return z; }

        vector<float> EgoCar::get_dimension() const {
            vector<float> dimension = {l, w, h};
            return dimension;
        }

        float EgoCar::get_yaw() const { return yaw; }

        float EgoCar::get_v() const { return velocity; }

        float EgoCar::get_a() const { return acc; }


}