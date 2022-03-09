#ifndef  DRAW_SHOW_H
#define  DRAW_SHOW_H

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
#include <algorithm>
#include <cmath>

using namespace std;
using namespace cv;

namespace riskfield {

class Draw {
    public:

        Draw(){}
        ~Draw(){}

        void updateMap2(cv::Mat& map, int i, int j, float val) {
            map.at<Vec3f>(i,j)[0] += val;
            map.at<Vec3f>(i,j)[1] += val;
            map.at<Vec3f>(i,j)[2] += val;
            // cout << i << " " << j << " " << map.at<Vec3f>(i,j) << endl;
        }

        void norm2draw(Mat& map) {

            int row = 0;
            int col = 0;
            // 防止map为空
            row = map.rows;
            col = map.cols;
            // cout << "map: " << row << " " << col << endl;
            assert(row > 0 && col > 0);

            for (int i = 0; i < row; i++) {
                for (int j = 0; j < col; ++j) {
                    max_E = max(max_E, map.at<Vec3f>(i,j)[0]);
                }
            }

            /** draw: range into 255 **/
            for (int i = 0; i < row; i++) {
                for (int j = 0; j < col; ++j) {
                    val = 255 * (map.at<Vec3f>(i,j)[0] / max_E);
                    updateMap2(map, i, j, val);
                }
            }
        }

    private:
        float max_E = 0.0;
        float val = 0.0;
};

};

#endif  //DRAW_SHOW_H