#include "car_risk_model.h"
#include "ego.hpp"

#include <iostream>
#include <cmath>
#include <exception>
#include <string>

namespace riskfield {

CarModel::CarModel(EgoCar *car) : ego(car) {}

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

void CarModel::carModel() {
    
}

}; // riskfield