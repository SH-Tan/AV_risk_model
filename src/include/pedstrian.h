/***
接受障碍物信息,
构建行车风险场.
姓名: Shuhang Tan
修改日期: 2022-5-12
需要参数:
    静态障碍物:
    动态障碍物:
***/

#ifndef __SMALL_DYNAMIC_RISk_MODEL_H
#define __SMALL_DYNAMIC_RISk_MODEL_H


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

#include "ped.hpp"

using namespace std;
using namespace cv;

namespace riskfield {

class Pedestrian {
    public:
        Pedestrian(const vector<ObsPed*> &ped_list);
        ~Pedestrian();

        /**v, dis, lambda, intend, TTC**/
        void pdeModel(Mat &map);

        double n1 = 0.4;
        double n2 = 1-n1;

        
    private:
        // Pedestrian vector
        vector<ObsPed*> ped_list_;   
};

}  // riskfield

#endif // __SMALL_DYNAMIC_RISk_MODEL_H
