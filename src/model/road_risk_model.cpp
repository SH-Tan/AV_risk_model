#include "road_risk_model.h"

#include <iostream>
#include <cmath>
#include <exception>
#include <string>
#include <vector>
#include <unordered_map>
#include <assert.h>


namespace riskfield {

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

vector<float> rotateAxisRoad(float x, float y, float r) {
    vector<float> ret(2);
    ret[0] = cos(r)*x + sin(r)*y;
    ret[1] = -sin(r)*x + cos(r)*y;
    return ret;
}

/**
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
    float Yl = 3*width/4 + s_w; // white lane
    float Yr = width/4 + s_w; // white lane
    float Yd = width/2 + s_w;  // yellow lane
    w.push_back(Yl);
    w.push_back(Yr);
    y.push_back(Yd);
    float val = 0.0;

    // yellow lane
    for (auto ye : y) {
        for (int i = s_w; i < s_w + width; i++) {
            double tmp = Ay*(exp(-pow((i-ye),2)/2*pow(a,2)));  // TODO error
            val = tmp > 0 ? tmp : 0.0;
            // cout << val << " ";
            for (int j = 0; j < col; ++j) {
                updateMap1(map, i, j, val);
            }
        }
        cout << endl;
    }

    // white lane
    for (auto wh : w) {
        for (int i = s_w; i < s_w + width; i++) {
            double tmp = Ay*(exp(-pow((i-wh),2)/2*pow(a,2)));
            val = tmp > 0 ? tmp : 0.0;
            for (int j = 0; j < col; ++j) {
                updateMap1(map, i, j, val);
            }
        }
    }
}
**/


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
    if (map.channels()!=3) {
        cout<< "should be 3 and not"<< map.channels()<<endl;
        return;
    }

    int row = 0;
    int col = 0;
    
    row = map.rows;
    col = map.cols;
    assert(row > 0 && col > 0);
    
    vector<float> axis;

    float angle = properties["yaw"];

    // need adjust
    float Yl = 3*properties["width"]/4 + properties["sw"];
    float Yr = properties["width"]/4 + properties["sw"];
    float val = 0.0;
    

    for (int i = properties["sw"]; i < properties["sw"] + properties["width"]*cos(angle); i++) {      
        for (int j = properties["sl"]; j < properties["sl"]+properties["length"]*cos(angle); ++j) {
            // axis = rotateAxisRoad(i, j, ego_yaw);  // ego's yaw
            double tmp = (exp(abs(i-Yl)/k)-1)/exp(properties["width"]*k) + (exp(abs(i-Yr)/k)-1)/exp(properties["width"]*k);
            val = tmp > 0 ? tmp : 0.0;
            updateMap1(map, i, j-i*tan(angle), val);
        }
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
    unordered_map<string, float> properties;
    for (auto lane : lane_list_) {

        string type = lane->get_type();
        properties["sl"] = lane->get_sl();
        properties["length"] = lane->get_l();
        properties["sw"] = lane->get_sw();
        properties["width"] = lane->get_w();
        properties["yaw"] = lane->get_yaw();

        if (type == "boundary") {

            BoundaryModel(map, properties);

        } else if (type == "white lane") {

            // WhiteModel(map, properties);

        } else if (type == "yellow lane") {

            // YellowModel(map, properties);

        } else {
            std::cout << "wrong lane type" << endl;
            return;
        }
    }
}

}  // riskfield
