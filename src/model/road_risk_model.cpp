#include "road_risk_model.h"

#include <iostream>
#include <cmath>
#include <exception>
#include <string>
#include <assert.h>


namespace riskfield {

RoadModel::RoadModel(float w, float st):width(w), s(st) {}
RoadModel::~RoadModel(){}

void updateMap1(cv::Mat& map, int i, int j, float val) {
    map.at<Vec3f>(i,j)[0] += val;
    map.at<Vec3f>(i,j)[1] += val;
    map.at<Vec3f>(i,j)[2] += val;
}

void RoadModel::LaneModel(cv::Mat &map, vector<float> y, vector<float> w) {

    if (map.empty()) {
        cout<< "Image is empty. Check file path";
        return;
    }
    if (map.channels()!=3) {
        cout<< "should be 3 and not"<< map.channels()<<endl;
        return;
    }

    int row = 0;
    int col = 0;
    // 防止map为空
    row = map.rows;
    col = map.cols;
    // cout << "map: " << row << " " << col << endl;
    assert(row > 0 && col > 0);

    // need adjust
    float Yl = 3*width/4 + s; // white lane
    float Yr = width/4 + s; // white lane
    float Yd = width/2 + s;  // yellow lane
    w.push_back(Yl);
    w.push_back(Yr);
    y.push_back(Yd);
    float val = 0.0;

    // yellow lane
    for (auto ye : y) {
        for (int i = s; i < s + width; i++) {
            int tmp = r*(exp(abs(i-Yl)/k)-1) + r*(exp(abs(i-Yr)/k)-1);
            val = tmp > 0 ? tmp : 0.0;
            for (int j = 0; j < col; ++j) {
                updateMap1(map, i, j, val);
            }
        }
    }

    // white lane
    for (auto wh : w) {
        for (int i = s; i < s + width; i++) {
            int tmp = r*(exp(abs(i-Yl)/k)-1) + r*(exp(abs(i-Yr)/k)-1);
            val = tmp > 0 ? tmp : 0.0;
            for (int j = 0; j < col; ++j) {
                updateMap1(map, i, j, val);
            }
        }
    }
    

    
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

    if (map.empty()) {
        cout<< "Image is empty. Check file path";
        return;
    }
    if (map.channels()!=3) {
        cout<< "should be 3 and not"<< map.channels()<<endl;
        return;
    }

    int row = 0;
    int col = 0;
    
    row = map.rows;
    col = map.cols;
    assert(row > 0 && col > 0);
      
    // need adjust
    float Yl = 3*width/4 + s;
    float Yr = width/4 + s;
    float val = 0.0;
    for (int i = s; i < s + width; i++) {
        int tmp = r*(exp(abs(i-Yl)/k)-1) + r*(exp(abs(i-Yr)/k)-1);
        val = tmp > 0 ? tmp : 0.0;
        for (int j = 0; j < col; ++j) {
            updateMap1(map, i, j, val);
        }
    }
}

};  // riskfield
