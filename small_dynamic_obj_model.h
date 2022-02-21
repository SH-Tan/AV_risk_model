/***
接受障碍物信息,
构建行车风险场.
姓名: Shuhang Tan
修改日期: 2022-2-21
需要参数:
    静态障碍物:
    动态障碍物:
***/

#ifndef __SMALL_DYNAMIC_RISk_MODEL_H
#define __SMALL_DYNAMIC_RISk_MODEL_H


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

class SmallObjModel {

};

}  // riskfield

#endif // __SMALL_DYNAMIC_RISk_MODEL_H
