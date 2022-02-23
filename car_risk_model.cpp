#include "car_risk_model.h"
#include "ego.hpp"
#include "obscar.hpp"

#include <iostream>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

using namespace std;

namespace riskfield {

CarModel::CarModel(EgoCar *car, const vector<ObsCar*> &obs_list):ego(car), obs_list_(obs_list) {}
CarModel::~CarModel() {
    if (ego != nullptr) {
        delete ego;
        ego = nullptr;
    }
}

float CarModel::cosine_similarity(float x, float y) {

}

float CarModel::calE(float tan_d, int k_a, float k_first) {

}

vector<float> CarModel::rotateAxis(float x, float y, float r) {

}

void CarModel::buildField() {

}

void CarModel::normV(vector<float> &vec) {

}

void CarModel::carModel() {
    int n = obs_list_.size(); // total obstacles
    for (auto obs_c : obs_list_) {
        vector<float> d_v = {(ego->get_x())-(obs_c->x), (ego->get_y())-(obs_c->y)};
        

    }
}

}; // riskfield