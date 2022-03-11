/***
接受障碍物信息,
构建行车风险场.
姓名: Shuhang Tan
修改日期: 2022-2-21
需要参数:
    ego car:
    道路几何信息:
***/

#ifndef __ROAD_RISk_MODEL_H
#define __ROAD_RISk_MODEL_H


#include <fstream> 

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <stdlib.h>

#include <mutex>
#include <thread>
#include <stack>
#include <vector>
#include <atomic>

using namespace std;
using namespace cv;

namespace riskfield {

class RoadModel {
    public:

        RoadModel(float w = 0, float st = 0);
        ~RoadModel();

        /*
         * args: 道路总宽, 车道线类型
         * 1. 双黄线或者道路中间物理分割等 yd
         * 2. 白色虚线: bd
         * 3. 白色实线: br
         * lane type: dict {'yd', num; 'bd', num; 'br', num}
         * ego_car struct
         */
        void LaneModel(cv::Mat &map, vector<float> y, vector<float> w);

        /*
         * args: 道路总宽, map, ego
         */
        void BoarderModel(cv::Mat &map);

    private:
        // Boarder const parameters
        // float r = 1/exp(160);
        const float k = 0.8;
        float width;  // 车道总宽
        float s = 0;  // 车道坐标开始

        // Lane const parameters
        const float Ay = exp(5);  // yellow
        const float Aw = exp(2);   // white
        const float a = 2;  // 控制幅度的参数

};

}  // riskfield

#endif // __ROAD_RISk_MODEL_H
