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
        
        static void refresh_ego(vector<float> dimension, vector<float> location, float y, float v, float a);

        static EgoCar* get_car();

        float get_x() const;

        float get_y() const;

        float get_z() const;

        vector<float> get_dimension() const;

        float get_yaw() const;

        float get_v() const;

        float get_a() const;


    private:
        EgoCar(vector<float> dimension = {0,0,0}, vector<float> location = {0,0,0}, float y = 0, float v = 0, float a = 0);  // 构造函数私有
        EgoCar(const EgoCar&) = delete;              // copy cotr 明确拒绝
        EgoCar& operator=(const EgoCar&) = delete;   // 明确拒绝 

        ~EgoCar(){}

        static atomic<EgoCar*> m_ego;

        static void update_ego (EgoCar* &car, vector<float> dimension, vector<float> location, float y, float v, float a);

        float l, w, h;  // long, width, height
        float x, y, z;  // location
        float yaw; // angle
        float velocity;
        float acc;
    
    private:
        class Deletor {
            public:
                ~Deletor() {
                    if(EgoCar::m_ego != nullptr) {
                        delete EgoCar::m_ego;
                        EgoCar::m_ego = nullptr;
                    }
                }
        };
        static Deletor deletor;
};

}  // riskfield

#endif  // __EGO_CAR_STRUCT_H
