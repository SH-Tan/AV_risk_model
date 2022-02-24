#include "car_risk_model.h"
#include "ego.hpp"
#include "obscar.hpp"

#include <iostream>
#include <cmath>
#include <exception>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <assert>

using namespace std;

namespace riskfield {

CarModel::CarModel(EgoCar *car, const vector<ObsCar*> &obs_list):ego(car), obs_list_(obs_list) {}
CarModel::~CarModel() {
    if (ego != nullptr) {
        delete ego;
        ego = nullptr;
    }
}

float getMold(const vector<float>& vec){   //求向量的模长
    int n = vec.size();
    float sum = 0.0;
    for (int i = 0; i<n; ++i)
        sum += vec[i] * vec[i];
    return sqrt(sum);
}

float CarModel::cosine_similarity(vector<float> x, vector<float> y) {
    assert(x.size() == y.size());
    float tmp = 0.0;  // 内积
    for (int i = 0; i < x.size(); ++i) {
        tmp += x[y]*y[i];
    }
    return tmp / (getMold(x)*getMold(y));
}

float CarModel::calE(float tan_d, int k_a, float k_first) {
    float k_E = exp(tan_d*k_a);
    float E = k_E / k_first;
    E_n = E / max(E);  // (0,1]
    return E_n;
}

vector<float> CarModel::rotateAxis(float x, float y, float r) {
    vector<float> ret(2);
    ret[0] = cos(r)*x + sin(r)*y;
    ret[1] = -sin(r)*x + cos(r)*y;
    return ret;
}

void CarModel::buildField() {

}

void CarModel::normV(vector<float> &vec) {
    float x = vec[0];
    float y = vec[1];

    float sum = sqrt(x * x + y * y);
    vec[0] = x/sum;
    vec[1] = y/sum;
}

void CarModel::carModel(cv::Mat &map) {
    int n = obs_list_.size(); // total obstacles
    for (auto obs_c : obs_list_) {
        vector<float> obs_d = obs_c->get_dimension();
        vector<float> d_v = {(ego->get_x())-(obs_c->get_x()), (ego->get_y())-(obs_c->get_y())};
        normV(d_v); // normalize vector

        // Obs 修正
        float v_re = obs_c->get_v() - ego->get_v();
        float yaw_re = obs_c->get_yaw() - ego->get_yaw();
        bool yaw_sign = yaw_re >= 0 ? true : false;

        // sample accelerate  Gaussian Distribution
        vector<float> acc_sample;
        unsigned seed = chrono::system_clock::now().time_since_epoch().count();
  
        default_random_engine generator(seed);
        // 第一个参数为高斯分布的平均值，第二个参数为标准差
        normal_distribution<float> distribution(0, 1);
        float obs_acc = obs_c->get_acc();
        for (int i = 0; i < 10; ++i) {
            float sam = distribution(generator)%2;
            assert(sam >= -2 && sam <= 2);
            acc_sample.push_back(obs_acc - sam);
        }

        // 角度修正后的单位向量 similarity calculate
        vector<float> yaw_vec = {cos(yaw_re), sin(yaw_re)};

        float similarity = cosine_similarity(d_v, yaw_vec);
        int similarity_sign = 1;
        if (similarity >= 0) {
            similarity_sign = 1;
        } else {
            similarity_sign = -1;
        }

        float k_d = 0.0;
        bool sign;
        // 严格跟驰状态
        // need modify 同一车道, 不产生变道\超车
        if (d_v[1] == 0) {
            sign = (yaw_re == 0) ? true : false;  // true 跟驰, 不发生变道
            float v_r = v_re * similarity_sign;

            if (!sign || (v_r < 0)) {  // 问题不大
                float ratio = abs(v_re)/ego->get_v();  // obj_v > 0, 一般不会大于2倍ego, ratio (0,1)
                k_d = (1+exp(-ratio)) * (obs_d[1]/obs_d[0]);  // (1.36,2)*w/l
            } else {
                k_d = 1 + log2(1+v_r);  // 横轴与垂直距离修正参数
            }
        } else {  // 非跟驰
            bool d_sign = d_v[1] > 0;  // 位置矢量, 角度逆时针
            sign = ~(d_sign ^ yaw_sign);  // true risk up
            k_d = 1;
            if (!sign) {
                float ratio = abs(v_re)/ego->get_v();  // obj_v > 0, 一般不会大于2倍ego, ratio (0,1)
                k_d = (1+exp(-ratio)) * (obs_d[1]/obs_d[0]);  // (1.36,2)*w/l
            } else {
                k_d = 1 + log2(1+v_revised)
            }

        }

        buildField();  // TODO
    }
}

}; // riskfield