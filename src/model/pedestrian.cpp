/**
 * @file small_dynamic_model.cpp
 * @Shuhang Tan (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-12
 * 
 * @copyright Copyright (c) 2022
 * 
 * 路口: 行人绿灯: 车: 1. 直行,等待; 2. 转弯,减速,有行人,等待
 * 行人红灯: 行人位置及速度: 1. 在道路边界之外: 车走; 2. 是否有进入道路的趋势(预测)
 *
 */

#include <iostream>
#include <cmath>
#include <exception>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <assert.h>
#include <algorithm>

#include "pedestrian.h"

using namespace std;
using namespace cv;

namespace riskfield {

// lambda formula
const vector<double> lambda_param = {3.8301, 0.0090, 25.7824, 0.0105, 18.3901, 0.7999, 5.7181, 617.3393}; 
vector<vector<double>> intensity;

Pedestrian::Pedestrian(const vector<ObsPed*> &ped_list): ped_list_(ped_list) {}

Pedestrian::~Pedestrian() {
    for (auto &p : ped_list_) {
        delete p;
    }
}

void updateMapPed(Mat &map, int i, int j, float val) {
    int row = map.rows;
    int col = map.cols;
    if(i >= 0 && i < row && j >= 0 && j < col) {
        map.at<Vec3f>(i,j)[0] += val;
        map.at<Vec3f>(i,j)[1] += val;
        map.at<Vec3f>(i,j)[2] += val;
    }
}

void getLambda() {

    intensity = new vector<vector<double>> (800, vector<double>(800, 0.0));

    for (int i = 0; i <= 70; i += 0.1) {
        for (int j =0; j <= 70; i += 0.1) {
            double intensity[i][j] = lambda_param[7]*pow(lambda_param[0]+lambda_param[1]*pow(i-lambda_param[2], 2)+
                lambda_param[3]*pow(j-lambda_param[4],2)-2*lambda_param[5]*(i-lambda_param[2])*
                    (j-lambda_param[4])*sqrt(lambda_param[1]*lambda_param[3]),(-lambda_param[6]));
        }
    }
}


void pedModel(Mat &map) {
    EgoCar* ego = EgoCar::get_car();  // get ego
    assert(ego != nullptr);
    double ego_v = ego->get_v();

    // calculate lambda 
    getLambda();

    for (auto ped : ped_list_) {
        // calculate distance
        double p_y = ped->get_y();
        double dis = abs(p_y-ego->get_v())/cos(ego->get_yaw());
        double ttc = dis/ego_v;

        for (int i = 0; i < intensity.size(); ++i) {
            for (int j = 0; j < intensity[0].size(); ++j) {
                double val = ped.n1*exp(intensity[i][j]) + ped.n2*exp(-ttc);
                updateMapPed(map, i, j, val);
            }
        }
    }
}
}  // riskfield



