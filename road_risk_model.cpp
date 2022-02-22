#include "road_risk_model.h"
#include <iostream>
#include <cmath>
#include <exception>
#include <string>

namespace riskfield {

void RoadModel::LaneModel(cv::Mat &map){
    int row = 0;
    int col = 0;
    // 防止map为空
    try {
        row = map.rows;
        col = map.cols;
    } catch (exception &e) {
        std::cout << "Build BoarderModel Fail, map is None" << std::endl;
        return;
    }

    // need adjust
    float Yl = 3*width/4;
    float Yr = width/4;


    
}

/**
 * @brief 
 * 
 * @param map 
 * ----------------------------->col j
 * |
 * |______________________________
 * |
 * |road
 * |______________________________
 * |
 * |
 * row i
 */

void RoadModel::BoarderModel(cv::Mat &map) {
    int row = 0;
    int col = 0;
    try {
        row = map.rows;
        col = map.cols;
    } catch (exception &e) {
        std::cout << "Build BoarderModel Fail, map is None" << std::endl;
        return;
    }
      
    // need adjust
    float Yl = 3*width/4;
    float Yr = width/4;

    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            map.at<float>(i,j) = r*(exp(abs(i-Yl)/k)-1) + r*(exp(abs(i-Yr)/k)-1);
        }
    }
}

};
