#include "road_risk_model.h"

#include <iostream>
#include <cmath>
#include <exception>
#include <string>
#include <assert.h>


namespace riskfield {

RoadModel::RoadModel(float w):width(w) {}
RoadModel::~RoadModel(){}

void updateMap1(cv::Mat& map, int i, int j, float val) {
    map.at<Vec3f>(i,j)[0] += val;
    map.at<Vec3f>(i,j)[1] += val;
    map.at<Vec3f>(i,j)[2] += val;
    // cout << i << " " << j << " " << map.at<Vec3f>(i,j) << endl;
}

void RoadModel::LaneModel(cv::Mat &map){

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
    float Yl = 3*width/4;
    float Yr = width/4;
    float Yd = width/2;
    float Ely = 0;
    float Elw = 0;
    float val = 0.0;
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            // cout << i << " " << j << endl;
            Ely = Ay * exp(-pow((i-Yd),2)/2*pow(a,2));
            Elw = Aw * exp(-pow(i-Yl,2)/2*pow(a,2)) + Aw * exp(-pow(i-Yr,2)/2*pow(a,2));
            val = Ely + Elw;
            updateMap1(map, i, j, val);
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
    int row = 0;
    int col = 0;
    
    row = map.rows;
    col = map.cols;
    assert(row > 0 && col > 0);
      
    // need adjust
    float Yl = 3*width/4;
    float Yr = width/4;
    float val = 0.0;
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            val = r*(exp(abs(i-Yl)/k)-1) + r*(exp(abs(i-Yr)/k)-1);
            updateMap1(map, i, j, val);
        }
    }
}

};
