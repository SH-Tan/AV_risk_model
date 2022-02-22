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


#include <math.h>
#include <ros/ros.h>
#include <fstream> 

#include <visualization_msgs/Marker.h>
#include "boundingbox.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include "GpsAndSendHelper.hpp"

#include <iostream>
#include <stdlib.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h> //多了这两行，不然会报错
#include <message_filters/sync_policies/approximate_time.h>

// MSG
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

//sensor_msgs
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>

#include <mutex>
#include <thread>
#include <stack>
#include <vector>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace darknet_ros_msgs;

namespace riskfield {

class RoadModel {
    public:

        RoadModel(float w) : width(w) {}
        ~RoadModel(){}

        /*
         * args: 道路总宽, 车道线类型
         * 1. 双黄线或者道路中间物理分割等 yd
         * 2. 白色虚线: bd
         * 3. 白色实线: br
         * lane type: dict {'yd', num; 'bd', num; 'br', num}
         * ego_car struct
         */
        void LaneModel(cv::Mat &map);

        /*
         * args: 道路总宽, map, ego
         */
        void BoarderModel(cv::Mat &map);

    private:
        // Boarder const parameters
        const float r = 1/exp(20);
        float width;  // 车道总宽

        // Lane const parameters
        const float Ay = exp(10);  // yellow
        const float Aw = exp(8);   // white
        const float a = 2;  // 控制幅度的参数

};

}  // riskfield

#endif // __ROAD_RISk_MODEL_H
