#include "car_risk_model.h"

#include <iostream>
#include <cmath>
#include <exception>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <assert.h>
#include <algorithm>

using namespace std;

namespace riskfield {

CarModel::CarModel(const vector<ObsCar*> &obs_list):obs_list_(obs_list) {}
CarModel::~CarModel() {
    for (auto o : obs_list_) {
        delete o;
    }
}

void updateMap(cv::Mat& map, int i, int j, float val) {
    map.at<Vec3f>(i,j)[0] = val;
    map.at<Vec3f>(i,j)[1] = val;
    map.at<Vec3f>(i,j)[2] = val;
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
    for (int i = 0; i < (int)x.size(); ++i) {
        tmp += x[i]*y[i];
    }
    return tmp / (getMold(x)*getMold(y));
}

float CarModel::calE(float tan_d, float a, float k_first) {
    float k_E = exp(tan_d*a);
    float E = k_E / k_first;

    return E;
}

vector<float> CarModel::rotateAxis(float x, float y, float r) {
    vector<float> ret(2);
    ret[0] = cos(r)*x + sin(r)*y;
    ret[1] = -sin(r)*x + cos(r)*y;
    return ret;
}

void CarModel::buildField(Mat &map, vector<float> location, vector<float> dimension, float k_d,
    vector<float> d_v, vector<float> k_a, float v, float yaw) {
        float x = location[0], y = location[1];
        float l = dimension[0], w = dimension[1];
        int row = map.rows;
        int col = map.cols;
        vector<float> axis;
        // float k;
        float k1_1;
        float k1_2;
        float x_d1;
        float x_d2; 
        float ss_1;
        float k_first;
        float tan_d;
        int n = k_a.size();
        float max_E = 1.0;
        float val = 0.0;
        for (int i = 0; i < row; ++i) {
            for (int j = 0; j < col; ++j) {
                axis = rotateAxis(i, j, yaw);
                for (int a : k_a) {
                    // k = exp(a*v);
                    k1_1 = l*k_d;
                    k1_2 = w;
                    x_d1 = pow((x-axis[0])/k1_1,2.0);
                    x_d2 = pow((y-axis[1])/k1_2,2.0); 
                    ss_1 = x_d2 + x_d1;
                    k_first = sqrt(ss_1) < 4 ? 4 : sqrt(ss_1);
                    tan_d = d_v[1]/d_v[0];
                    val += calE(tan_d, a, k_first)/n;
                }
                updateMap(map, i, j, val);
                val = 0.0;
                max_E = max(max_E, map.at<Vec3f>(i,j)[0]);

            }
        }

        // 归一化
        if (max_E > 1) {
            for (int i = 0; i < row; ++i) {
                for (int j = 0; j < col; ++j) {
                    val = map.at<Vec3f>(i,j)[0] / max_E;
                    updateMap(map, i, j, val);
                }
            }
        }
        

}

void CarModel::normV(vector<float> &vec) {
    float x = vec[0];
    float y = vec[1];

    float sum = sqrt(x * x + y * y);
    vec[0] = x/sum;
    vec[1] = y/sum;
}

void CarModel::carModel(cv::Mat &map) {
    EgoCar* ego = EgoCar::get_car();
    assert(ego != nullptr);
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
        float obs_acc = obs_c->get_a();
        normal_distribution<float> distribution(obs_acc, 1);

        for (int i = 0; i < 10; ++i) {
            float sam = distribution(generator);
            // assert(sam >= -2 && sam <= 2);
            acc_sample.push_back(sam);
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
                k_d = 1 + log2(1+v_re);
            }

        }


        vector<float> lo = {obs_c->get_x(), obs_c->get_y(), obs_c->get_z()};
        buildField(map, lo, obs_d, k_d,
            d_v, acc_sample, obs_c->get_v(), obs_c->get_yaw());  // TODO
    }
}

};  // riskfield