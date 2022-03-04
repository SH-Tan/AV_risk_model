#ifndef __EGO_CAR_STRUCT_H
#define __EGO_CAR_STRUCT_H

#include <cmath>
#include <vector>
#include <mutex>
#include <atomic>


using namespace std;

namespace riskfield {

static mutex mtx;

class EgoCar {
    public:
        
        static void refresh_ego(vector<float> dimension, vector<float> location, float y, float v, float a) {

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

        static EgoCar* get_car() {
            if (m_ego != nullptr) {
                return m_ego;
            }
            return nullptr;
        }

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
        EgoCar(vector<float> dimension = {0,0,0}, vector<float> location = {0,0,0}, float y = 0, float v = 0, float a = 0)
            : l(dimension[0]), w(dimension[1]), h(dimension[2]),
              x(location[0]), y(location[1]), z(location[2]),
              yaw(y), velocity(v), acc(a) {}         // 构造函数私有
        EgoCar(const EgoCar&) = delete;              // copy cotr 明确拒绝
        EgoCar& operator=(const EgoCar&) = delete;   // 明确拒绝 

        ~EgoCar(){}

        static atomic<EgoCar*> m_ego;

        static void update_ego (EgoCar* &car, vector<float> dimension, vector<float> location, float y, float v, float a) {
            car->l = dimension[0];
            car->w = dimension[1];
            car->h = dimension[2];
            car->x = location[0];
            car->y = location[1];
            car->z = location[2];
            car->yaw = y;
            car->velocity = v;
            car->acc = a;
        };

        float l, w, h;  // long, width, height
        float x, y, z;  // location
        float yaw; // angle
        float velocity;
        float acc;
    
    private:
        class Deletor {
            public:
                ~Deletor() {
                    if(EgoCar::m_ego != nullptr)
                        delete EgoCar::m_ego;
                }
        };
        static Deletor deletor;

};

atomic<EgoCar*> EgoCar::m_ego{nullptr};
EgoCar::Deletor EgoCar::deletor;

};  // riskfield

#endif  // __EGO_CAR_STRUCT_H

