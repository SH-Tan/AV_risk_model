#include "road_risk_model.h"

#include <iostream>
#include <cmath>
#include <exception>
#include <string>
#include <vector>
#include <unordered_map>
#include <assert.h>


namespace riskfield {
// static int index = 0;

RoadModel::RoadModel(vector<Lane*> l):lane_list_(l) {}
RoadModel::~RoadModel(){
    for (auto l : lane_list_) {
        delete l;
    }
}

void updateMap1(cv::Mat& map, int i, int j, float val) {
    map.at<Vec3f>(i,j)[0] += val;
    map.at<Vec3f>(i,j)[1] += val;
    map.at<Vec3f>(i,j)[2] += val;
}


void RoadModel::YellowModel(cv::Mat &map, unordered_map<string, float> &properties) {
    if (map.empty()) {
        cout<< "Image is empty. Check file path";
        return;
    }

    int row = 0;
    int col = 0;
    // 防止map为空
    row = map.rows;
    col = map.cols;
    assert(row > 0 && col > 0);

    float angle = -properties["yaw"];

    float ye =  (properties["st"]- properties["sw"])*cos(angle)*cos(angle) + properties["sw"];
    float val = 0.0;

    float b = 0;

    for (int i = 0; i < properties["width"]; i++) {

        b = (i+properties["sw"]) - tan(angle)*properties["sl"];
        float cur = i*cos(angle)*cos(angle) + properties["sw"];
        double tmp = Ay*(exp(-pow((cur-ye),2)/2*pow(a,2)));

        // cout << "ye = " << ye << " cur = " << cur << endl;
        // cout << "pow = " << pow((cur-ye),2)/(2*pow(a,2)) << " diff = " << (cur-ye) << endl;

        val = tmp > 0 ? tmp : 0.0;
        // cout << val << "  ";
        for (int j = properties["sl"]; j < properties["sl"]+properties["length"]*cos(angle); ++j) {
            updateMap1(map, j*tan(angle)+b, j, val);
        }
    }
}


void RoadModel::WhiteModel(cv::Mat &map, unordered_map<string, float> &properties) {
    if (map.empty()) {
        cout<< "Image is empty. Check file path";
        return;
    }

    int row = 0;
    int col = 0;
    // 防止map为空
    row = map.rows;
    col = map.cols;
    // cout << "map: " << row << " " << col << endl;
    assert(row > 0 && col > 0);

    float angle = -properties["yaw"];

    float wh =  (properties["st"]- properties["sw"])*cos(angle)*cos(angle) + properties["sw"];
    float val = 0.0;

    float b = 0;

    for (int i = 0; i < properties["width"]; i++) {

        b = (i+properties["sw"]) - tan(angle)*properties["sl"];
        float cur = i*cos(angle)*cos(angle) + properties["sw"];
        cout << "wh = " << wh << " cur = " << cur << endl;
        cout << "pow = " << pow((cur-wh),2)/(2*pow(a,2)) << " diff = " << (cur-wh) << endl;
        double tmp = Aw*(exp(-pow((cur-wh),2)/(2*pow(a,2))));
        val = tmp > 0 ? tmp : 0.0;
        // cout << val << "  ";
        for (int j = properties["sl"]; j < properties["sl"]+properties["length"]*cos(angle); ++j) {
            updateMap1(map, j*tan(angle)+b, j, val);
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

void RoadModel::BoundaryModel(cv::Mat &map, unordered_map<string, float> &properties) {

    if (map.empty()) {
        cout<< "Image is empty. Check file path";
        return;
    }

    int row = 0;
    int col = 0;
    
    row = map.rows;
    col = map.cols;
    assert(row > 0 && col > 0);

    float angle = -properties["yaw"];

    float Yl = (3*properties["width"]*cos(angle)/4)*cos(angle) + properties["sw"];
    float Yr = (properties["width"]*cos(angle)/4)*cos(angle) + properties["sw"];
    float val = 0.0;

    float b = 0;

    for (int i = 0; i < properties["width"]; i++) {      
        b = (i+properties["sw"]) - tan(angle)*properties["sl"];
        float cur = i*cos(angle)*cos(angle)+ properties["sw"];
        // cout << cur << " ";
        double tmp = (exp(abs(cur-Yl)/k)-1)/exp(Yl-properties["sw"]) + (exp(abs(cur-Yr)/k)-1)/exp(Yl-properties["sw"]);
        // cout << tan(angle) << " " << b << " " << tmp;
        // cout << b << " ";
        for (int j = properties["sl"]; j < properties["sl"]+properties["length"]*cos(angle); ++j) {
            // axis = rotateAxisRoad(i, j, ego_yaw);  // ego's yaw
            val = tmp > 0 ? tmp : 0.0;
            updateMap1(map, j*tan(angle)+b, j, val);
        }
        // cout << endl;
    }

}

/*
 * float sl;  // 长度开始坐标,图像坐标系
 * float length;  // 长度
 * float sw;  // 宽度开始坐标,图像坐标系
 * float width;  // 宽度
 * float yaw; // angle
 * string type;
 */
void RoadModel::buildLaneModel(cv::Mat &map) {
    /** draw **/
    // Draw *draw_pen = new Draw();

    unordered_map<string, float> properties;

    // for (auto lane : lane_list_) 
    for (int i = 0; i < (int)lane_list_.size(); i++) {
        Lane* lane = lane_list_[i];
        string type = lane->get_type();
        properties["sl"] = lane->get_sl();
        properties["length"] = lane->get_l();
        properties["sw"] = lane->get_sw();
        properties["width"] = lane->get_w();
        properties["yaw"] = lane->get_yaw();
        properties["st"] = lane->get_st();

        if (type == "boundary") {
            
            BoundaryModel(map, properties);

        } else if (type == "white lane") {
            // cout << "white lane" << endl;
            WhiteModel(map, properties);

        } else if (type == "yellow lane") {

            YellowModel(map, properties);

        } else {
            std::cout << "wrong lane type" << endl;
            return;
        }
    }
}

}  // riskfield
