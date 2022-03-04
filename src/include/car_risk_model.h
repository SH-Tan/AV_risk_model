/***
接受障碍物信息,
构建行车风险场.
姓名: Shuhang Tan
修改日期: 2022-2-23
需要参数:
    静态障碍物:
    动态障碍物:
***/

#ifndef __CAR_RISk_MODEL_H
#define __CAR_RISk_MODEL_H


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

// ego
#include "ego.hpp"
#include "obscar.hpp"

using namespace std;
using namespace cv;

namespace riskfield {

class CarModel {
    public:
        CarModel(const vector<ObsCar*> &obs_list);
        ~CarModel();

        /**
         * @brief calculate the cos similarity between x and y
         * 
         * @param x 
         * @param y 
         * @return float 
         */
        float cosine_similarity(vector<float> x, vector<float> y);

        /**
         * @brief 
         * 
         * @param tan_d 
         * @param k_a 
         * @param k_first 
         * @return float 
         */
        float calE(float tan_d, float a, float k_first);

        /**
         * @brief convert the angle
         * 
         * @param x 
         * @param y 
         * @param r 
         * @return vector<float> 
         */
        vector<float> rotateAxis(float x, float y, float r);

        /**
         * @brief 
         * 向量大小归一化
         * @param vec 
         */
        void normV(vector<float> &vec);

        /**
         * @brief 
         * 
         * @param location 
         * @param dimension 
         * @param k_d 
         * @param d_v 
         * @param k_a 
         * @param v_r 
         * @param yaw 
         */
        void buildField(Mat &map, vector<float> location, vector<float> dimension, float k_d,
                vector<float> d_v, vector<float> k_a, float v_r, float yaw);

        /**
         * @brief 
         * 入口函数
         */
        void carModel(cv::Mat &map);


    private:
        vector<ObsCar*> obs_list_;
};

}  // riskfield

#endif // __CAR_RISk_MODEL_H
